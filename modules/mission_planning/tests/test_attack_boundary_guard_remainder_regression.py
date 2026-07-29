"""Scenario 2026-07-29T112020 boundary-AREA attack regressions.

That run reached attack replanning with three independently trimmed Type-2
boundary guard sets.  Their source/remaining child counts were 5 -> 3 for UAV4,
4 -> 3 for UAV5, and 4 -> 2 for UAV6.  Stale sequence metadata made attack
tracking fail with ``inconsistent sequence count`` and made the paired attack
exclusion graph fail validation as well.

These tests keep those exact shapes executable.  They also prove that repairing
the UAV boundary loops does not alter or lose the already-generated LAH attack
waypoint.
"""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    BOUNDARY_GUARD_CONTRACT_KEYS,
    apply_boundary_guard_contract,
    boundary_guard_contract,
    extract_boundary_guard_contract,
    finalize_boundary_guard_flight_path_sets_in_mission_order,
    validate_boundary_guard_flight_path_sets,
)
from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
)
from modules.mission_planning.runtime.validation.replan_payloads import (
    ReplanValidationError,
    validate_generated_artifact_payloads,
)


_SCENARIO_REMAINDERS = (
    # aircraftID, source child count, surviving source sequences
    (4, 5, (3, 4, 5)),
    (5, 4, (2, 3, 4)),
    (6, 4, (3, 4)),
)


def _waypoint(waypoint_id: int, next_waypoint_id: int) -> dict[str, Any]:
    return {
        "waypointID": int(waypoint_id),
        "coordinate": {
            "latitude": 38.0 + int(waypoint_id) * 1e-8,
            "longitude": 127.0,
            "altitude": 1000,
        },
        "speed": 40.0,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": int(next_waypoint_id),
        "waypointPassType": 1,
    }


def _guard_remainder(
    aircraft_id: int,
    source_count: int,
    source_sequences: tuple[int, ...],
    *,
    token: int = 0,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    set_id = f"type2-boundary:3:5:aircraft-{aircraft_id}"
    missions: list[dict[str, Any]] = []
    paths: list[dict[str, Any]] = []
    for offset, source_sequence in enumerate(source_sequences, start=1):
        path_id = int(aircraft_id) * 100_000_000 + 10_000 + token * 100 + offset
        mission_id = 900_100_000 + int(aircraft_id) * 1_000 + token * 100 + offset
        first_waypoint_id = 100_000 + token * 10_000 + int(aircraft_id) * 100 + offset * 2
        contract = boundary_guard_contract(
            set_id=set_id,
            sequence=int(source_sequence),
            sequence_count=int(source_count),
            duration_s=600,
        )
        mission = {
            "aircraftID": int(aircraft_id),
            "individualMissionID": int(mission_id),
            "isDone": False,
            "relatedMission": {
                "relatedMissionType": 1,
                "inputMissionID": 5,
                "priorMissionID": 0,
            },
            "individualMissionInfo": {
                "individualMissionType": 5,
                "patternType": 1,
                "autoZoomIn": True,
                "coordinateList": [
                    {"latitude": 38.0, "longitude": 127.0, "altitude": 1000}
                ],
                "targetID": None,
            },
            "pathID": int(path_id),
        }
        path = {
            "pathID": int(path_id),
            "aircraftID": int(aircraft_id),
            "individualMissionID": int(mission_id),
            "waypointList": [
                _waypoint(first_waypoint_id, first_waypoint_id + 1),
                _waypoint(first_waypoint_id + 1, 0),
            ],
        }
        apply_boundary_guard_contract(
            mission,
            contract,
            include_individual_mission_info=True,
        )
        apply_boundary_guard_contract(path, contract)
        missions.append(mission)
        paths.append(path)
    return (
        {
            "individualMissionPackageID": 800_100_000 + int(aircraft_id) + token * 10,
            "aircraftID": int(aircraft_id),
            "individualMissionList": missions,
        },
        paths,
    )


def _attack_artifacts() -> tuple[dict[str, Any], dict[str, Any]]:
    mission_id = 900_200_002
    path_id = 200_020_002
    package = {
        "individualMissionPackageID": 800_200_002,
        "aircraftID": 2,
        "individualMissionList": [
            {
                "aircraftID": 2,
                "individualMissionID": mission_id,
                "isDone": False,
                "individualMissionInfo": {
                    "individualMissionType": 2,
                    "targetID": 13,
                },
                "pathID": path_id,
            }
        ],
    }
    path = {
        "pathID": path_id,
        "aircraftID": 2,
        "individualMissionID": mission_id,
        "lahWaypointList": [
            {
                **_waypoint(200_001, 0),
                "attack": {"targetID": 13, "weaponType": 2},
            }
        ],
    }
    return package, path


def _finalize_guard_rows(
    package: dict[str, Any],
    paths: list[dict[str, Any]],
) -> None:
    guard_missions = [
        mission
        for mission in package["individualMissionList"]
        if extract_boundary_guard_contract(
            mission,
            mission.get("individualMissionInfo"),
        )
    ]
    guard_paths = [path for path in paths if extract_boundary_guard_contract(path)]
    finalize_boundary_guard_flight_path_sets_in_mission_order(
        guard_missions,
        guard_paths,
        strict=True,
    )


def test_attack_tracking_repairs_exact_112020_remainders_and_keeps_attack_wp() -> None:
    """The attack graph survives the exact 5->3, 4->3, and 4->2 trims."""

    attack_package, attack_path = _attack_artifacts()
    attack_path_before = deepcopy(attack_path)
    packages = [attack_package]
    paths = [attack_path]

    for aircraft_id, source_count, source_sequences in _SCENARIO_REMAINDERS:
        package, guard_paths = _guard_remainder(
            aircraft_id,
            source_count,
            source_sequences,
        )
        packages.append(package)
        paths.extend(guard_paths)

    with pytest.raises(ReplanValidationError, match="boundary guard set"):
        validate_generated_artifact_payloads(
            individual_mission_plans=packages,
            flight_paths=paths,
            allow_existing_db_artifacts=False,
            scope="attack112020BeforeFinalization",
        )

    for package in packages[1:]:
        package_path_ids = {
            int(mission["pathID"])
            for mission in package["individualMissionList"]
        }
        _finalize_guard_rows(
            package,
            [path for path in paths if int(path["pathID"]) in package_path_ids],
        )

    summary = validate_generated_artifact_payloads(
        individual_mission_plans=packages,
        flight_paths=paths,
        allow_existing_db_artifacts=False,
        scope="attack112020AfterFinalization",
    )
    assert summary["individualMissionPackages"] == 4

    for package, (_aircraft_id, _source_count, source_sequences) in zip(
        packages[1:],
        _SCENARIO_REMAINDERS,
    ):
        assert [
            int(mission["boundaryGuardSequence"])
            for mission in package["individualMissionList"]
        ] == list(range(1, len(source_sequences) + 1))
        assert {
            int(mission["boundaryGuardSequenceCount"])
            for mission in package["individualMissionList"]
        } == {len(source_sequences)}

    # Guard finalization is scoped to UAV AREA children.  It must not mutate
    # or accidentally remove the executable LAH attack waypoint.
    assert attack_path == attack_path_before
    mission_plan = {
        "missionPlanID": 700_200_002,
        "aircraftList": [
            {
                "aircraftID": int(package["aircraftID"]),
                "individualMissionPackageID": int(
                    package["individualMissionPackageID"]
                ),
            }
            for package in packages
        ],
    }
    attack_rows, errors = collect_lah_attack_rows(
        mission_plan,
        individual_mission_plans=packages,
        flight_paths=paths,
    )
    assert errors == []
    assert {
        (int(row["aircraftID"]), int(row["targetID"])) for row in attack_rows
    } == {(2, 13)}


class _FreshIdReservation:
    def __init__(self) -> None:
        self._imp = 800_300_000
        self._mission = 900_300_000
        self._waypoint = 300_000
        self._path_by_aircraft = {4: 400_030_000, 5: 500_030_000, 6: 600_030_000}

    def next_imp(self) -> int:
        self._imp += 1
        return self._imp

    def next_individual(self) -> int:
        self._mission += 1
        return self._mission

    def next_path(self, aircraft_id: int) -> int:
        self._path_by_aircraft[int(aircraft_id)] += 1
        return self._path_by_aircraft[int(aircraft_id)]

    def next_waypoint(self) -> int:
        self._waypoint += 1
        return self._waypoint

    def summary(self) -> dict[str, Any]:
        return {"test": "scenario-112020"}


def test_attack_exclusion_fresh_ids_repairs_exact_112020_remainders(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Exercise the real attack-exclusion fresh-ID/final-validation path."""

    plan = {"missionPlanID": 700_300_001, "aircraftList": []}
    source_packages: list[dict[str, Any]] = []
    for aircraft_id, source_count, source_sequences in _SCENARIO_REMAINDERS:
        package, paths = _guard_remainder(
            aircraft_id,
            source_count,
            source_sequences,
            token=1,
        )
        source_packages.append(package)
        plan["aircraftList"].append(
            {
                "aircraftID": int(aircraft_id),
                "individualMissionPackageID": int(
                    package["individualMissionPackageID"]
                ),
            }
        )
        imp_path = (
            tmp_path
            / "IndividualMissionPlan"
            / f"{int(package['individualMissionPackageID'])}.json"
        )
        imp_path.parent.mkdir(parents=True, exist_ok=True)
        imp_path.write_text(json.dumps(package), encoding="utf-8")
        for path in paths:
            path_file = tmp_path / "FlightPath" / f"{int(path['pathID'])}.json"
            path_file.parent.mkdir(parents=True, exist_ok=True)
            path_file.write_text(json.dumps(path), encoding="utf-8")

    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )
    monkeypatch.setattr(
        attack,
        "read_json_cached",
        lambda path, **_kwargs: json.loads(Path(path).read_text(encoding="utf-8")),
    )
    written: list[tuple[Path, dict[str, Any]]] = []

    def capture_writes(entries: Any) -> list[dict[str, Any]]:
        rows = [(Path(path), deepcopy(payload)) for path, payload in entries]
        written.extend(rows)
        return [{"path": str(path), "written": True} for path, _payload in rows]

    monkeypatch.setattr(attack, "_write_json_files_batch", capture_writes)

    result = attack._freshen_attack_exclusion_artifact_ids(
        plan,
        now_ms=1_785_291_900_000,
        emit=lambda _message: None,
        id_reservation=_FreshIdReservation(),
    )

    cloned_packages = [
        payload
        for _path, payload in written
        if "individualMissionPackageID" in payload
    ]
    cloned_paths = [payload for _path, payload in written if "pathID" in payload]
    assert result["individualMissionCount"] == 8
    assert result["pathCount"] == 8
    assert len(cloned_packages) == 3
    assert len(cloned_paths) == 8

    # The production helper validates before this write hook.  Validate once
    # more from the captured graph and pin the exact post-trim cardinalities.
    validate_generated_artifact_payloads(
        individual_mission_plans=cloned_packages,
        flight_paths=cloned_paths,
        allow_existing_db_artifacts=False,
        scope="attackExclusion112020FreshIds",
    )
    for package, (_aircraft_id, _source_count, source_sequences) in zip(
        sorted(cloned_packages, key=lambda row: int(row["aircraftID"])),
        _SCENARIO_REMAINDERS,
    ):
        missions = package["individualMissionList"]
        assert [int(row["boundaryGuardSequence"]) for row in missions] == list(
            range(1, len(source_sequences) + 1)
        )
        assert {int(row["boundaryGuardSequenceCount"]) for row in missions} == {
            len(source_sequences)
        }


class _AttackResumeReservation:
    def __init__(self) -> None:
        self._waypoint = 400_100
        self._individual = 900_400_010
        self._path = 600_040_010

    def next_paths(self, aircraft_id: int, count: int) -> list[int]:
        assert int(aircraft_id) == 6
        assert int(count) == 2
        return [600_040_001, 600_040_002]

    def next_individuals(self, count: int) -> list[int]:
        assert int(count) == 1
        return [900_400_002]

    def next_individual(self) -> int:
        self._individual += 1
        return self._individual

    def next_path(self, aircraft_id: int) -> int:
        assert int(aircraft_id) == 6
        self._path += 1
        return self._path

    def next_waypoint(self) -> int:
        self._waypoint += 1
        return self._waypoint


class _AttackTrackingReservation:
    def __init__(self) -> None:
        self._waypoint = 550_100
        self._individual = 900_550_010
        self._path = 400_055_010

    def next_paths(self, aircraft_id: int, count: int) -> list[int]:
        assert int(aircraft_id) == 4
        assert int(count) == 2
        return [400_055_001, 400_055_002]

    def next_individuals(self, count: int) -> list[int]:
        assert int(count) == 2
        return [900_550_001, 900_550_002]

    def next_individual(self) -> int:
        self._individual += 1
        return self._individual

    def next_path(self, aircraft_id: int) -> int:
        assert int(aircraft_id) == 4
        self._path += 1
        return self._path

    def next_waypoint(self) -> int:
        self._waypoint += 1
        return self._waypoint


def test_uav_attack_resume_builder_repairs_uav6_four_to_two_guard_cycle(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Directly cover the non-tracking UAV resume builder from 112020.

    UAV6 resumes the current original sequence 3 and retains original sequence
    4 as its follow-up.  Both still declare the old four-child owner set when
    they enter the builder.  Store-time validation must see a rebuilt 1..2
    cycle, not the stale 3/4-of-4 contract.
    """

    source_package, source_paths = _guard_remainder(6, 4, (3, 4), token=2)
    current_mission = deepcopy(source_package["individualMissionList"][0])
    source_follow_up_mission = deepcopy(
        source_package["individualMissionList"][1]
    )
    current_path = deepcopy(source_paths[0])
    source_follow_up_path = deepcopy(source_paths[1])

    resumed_waypoints = [
        _waypoint(400_201, 400_202),
        _waypoint(400_202, 0),
    ]
    follow_up_mission = deepcopy(source_follow_up_mission)
    follow_up_mission["individualMissionID"] = 900_400_003
    follow_up_mission["pathID"] = 600_040_003
    follow_up_path = deepcopy(source_follow_up_path)
    follow_up_path["individualMissionID"] = 900_400_003
    follow_up_path["pathID"] = 600_040_003
    follow_up_path["waypointList"] = [
        _waypoint(400_203, 400_204),
        _waypoint(400_204, 0),
    ]

    monkeypatch.setattr(
        attack,
        "_source_type2_self_reliance_phase",
        lambda **_kwargs: attack.TYPE2_SELF_RELIANCE_GUARD_AREA,
    )
    monkeypatch.setattr(
        attack,
        "_split_done_resume_path",
        lambda *_args, **_kwargs: ([], deepcopy(resumed_waypoints), 400_200),
    )
    monkeypatch.setattr(
        attack,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [deepcopy(follow_up_mission)],
            [
                (
                    tmp_path / "FlightPath" / "600040003.json",
                    deepcopy(follow_up_path),
                )
            ],
            {"preserved": 1},
        ),
    )
    monkeypatch.setattr(
        attack,
        "_trim_uav_follow_up_paths_after_anchor",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        attack,
        "_sync_resume_mission_info_with_waypoints",
        lambda *_args, **_kwargs: False,
    )
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )

    captured_entries: list[tuple[Path, dict[str, Any]]] = []

    def capture_deferred_writes(
        entries: Any,
        *,
        defer_write: bool,
    ) -> tuple[list[dict[str, Any]], list[tuple[Path, dict[str, Any]]]]:
        assert defer_write is True
        rows = [(Path(path), deepcopy(payload)) for path, payload in entries]
        captured_entries.extend(rows)
        return (
            [
                {"path": str(path), "written": False, "skipped": True}
                for path, _payload in rows
            ],
            rows,
        )

    monkeypatch.setattr(
        attack,
        "_write_or_defer_attack_json_entries",
        capture_deferred_writes,
    )

    result = attack._build_uav_attack_resume_package(
        descriptor={"aircraft_id": 6, "label": "uav_resume_6"},
        new_imp_id=800_400_002,
        imp_data=deepcopy(source_package),
        fp_data=current_path,
        target_mission_template=current_mission,
        target_index=0,
        ctx={"mission_ids": [5]},
        state={
            "coordinate": {
                "latitude": 38.0,
                "longitude": 127.0,
                "altitude": 1000,
            },
            "heading": 0.0,
            "speed": 40.0,
        },
        artifacts=SimpleNamespace(source_plan_id=700_400_001),
        emit=lambda _message: None,
        now_ms=1_785_291_901_000,
        done_input_ids=set(),
        id_reservation=_AttackResumeReservation(),
        defer_write=True,
    )

    assert result is not None
    generated_package = next(
        payload
        for _path, payload in captured_entries
        if "individualMissionPackageID" in payload
    )
    generated_paths = [
        payload for _path, payload in captured_entries if "pathID" in payload
    ]
    generated_missions = generated_package["individualMissionList"]

    assert [int(row["boundaryGuardSequence"]) for row in generated_missions] == [
        1,
        2,
    ]
    assert {
        int(row["boundaryGuardSequenceCount"]) for row in generated_missions
    } == {2}
    assert [int(row["boundaryGuardSequence"]) for row in generated_paths] == [1, 2]
    assert {int(row["boundaryGuardSequenceCount"]) for row in generated_paths} == {
        2
    }
    assert generated_paths[0]["waypointList"][-1]["nextWaypointID"] == 400_203
    assert generated_paths[1]["waypointList"][-1]["nextWaypointID"] == 400_201

    validate_generated_artifact_payloads(
        individual_mission_plans=[generated_package],
        flight_paths=generated_paths,
        allow_existing_db_artifacts=False,
        scope="attackUavResume112020Uav6FourToTwo",
    )


def test_uav_attack_resume_keeps_later_guard_child_when_current_child_is_empty(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An empty current child must not replace a pending guard child with loiter."""

    source_package, source_paths = _guard_remainder(6, 4, (3, 4), token=3)
    current_mission = deepcopy(source_package["individualMissionList"][0])
    follow_up_mission = deepcopy(source_package["individualMissionList"][1])
    current_path = deepcopy(source_paths[0])
    follow_up_path = deepcopy(source_paths[1])
    follow_up_mission["individualMissionID"] = 900_500_003
    follow_up_mission["pathID"] = 600_050_003
    follow_up_path["individualMissionID"] = 900_500_003
    follow_up_path["pathID"] = 600_050_003
    follow_up_path["waypointList"] = [
        _waypoint(500_203, 500_204),
        _waypoint(500_204, 0),
    ]

    monkeypatch.setattr(
        attack,
        "_source_type2_self_reliance_phase",
        lambda **_kwargs: attack.TYPE2_SELF_RELIANCE_GUARD_AREA,
    )
    monkeypatch.setattr(
        attack,
        "_split_done_resume_path",
        lambda *_args, **_kwargs: ([], [], 500_200),
    )

    def fail_if_hold_is_built(*_args: Any, **_kwargs: Any) -> None:
        raise AssertionError("pending boundary child must suppress synthetic hold")

    monkeypatch.setattr(
        attack,
        "_build_uav_attack_completion_hold_waypoint",
        fail_if_hold_is_built,
    )
    monkeypatch.setattr(
        attack,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [deepcopy(follow_up_mission)],
            [
                (
                    tmp_path / "FlightPath" / "600050003.json",
                    deepcopy(follow_up_path),
                )
            ],
            {"clonedCount": 1},
        ),
    )
    monkeypatch.setattr(
        attack,
        "_trim_uav_follow_up_paths_after_anchor",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )

    captured_entries: list[tuple[Path, dict[str, Any]]] = []

    def capture_deferred_writes(
        entries: Any,
        *,
        defer_write: bool,
    ) -> tuple[list[dict[str, Any]], list[tuple[Path, dict[str, Any]]]]:
        assert defer_write is True
        rows = [(Path(path), deepcopy(payload)) for path, payload in entries]
        captured_entries.extend(rows)
        return ([{"path": str(path), "written": False} for path, _ in rows], rows)

    monkeypatch.setattr(
        attack,
        "_write_or_defer_attack_json_entries",
        capture_deferred_writes,
    )

    result = attack._build_uav_attack_resume_package(
        descriptor={"aircraft_id": 6, "label": "uav_resume_6"},
        new_imp_id=800_500_002,
        imp_data=deepcopy(source_package),
        fp_data=current_path,
        target_mission_template=current_mission,
        target_index=0,
        ctx={"mission_ids": [5]},
        state={
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
            "heading": 0.0,
            "speed": 40.0,
        },
        artifacts=SimpleNamespace(source_plan_id=700_500_001),
        emit=lambda _message: None,
        now_ms=1_785_291_902_000,
        # Reproduce the risky case where input-level progress says done while a
        # later child in the owner set is still pending.
        done_input_ids={5},
        id_reservation=_AttackResumeReservation(),
        defer_write=True,
    )

    assert result is not None
    assert result["completionBoundaryHold"] is False
    assert result["resumePath"] is None
    generated_package = next(
        payload
        for _path, payload in captured_entries
        if "individualMissionPackageID" in payload
    )
    generated_paths = [
        payload for _path, payload in captured_entries if "pathID" in payload
    ]
    assert len(generated_package["individualMissionList"]) == 1
    assert len(generated_paths) == 1
    generated_mission = generated_package["individualMissionList"][0]
    generated_path = generated_paths[0]
    assert generated_mission["boundaryGuardSequence"] == 1
    assert generated_mission["boundaryGuardSequenceCount"] == 1
    assert generated_path["boundaryGuardSequence"] == 1
    assert generated_path["boundaryGuardSequenceCount"] == 1
    assert generated_path["waypointList"][-1]["nextWaypointID"] == 500_203
    assert generated_path["waypointList"][-1]["eta"] == 600
    validate_generated_artifact_payloads(
        individual_mission_plans=[generated_package],
        flight_paths=generated_paths,
        allow_existing_db_artifacts=False,
        scope="attackUavResumePendingGuardChildWithoutHold",
    )


def test_uav_attack_tracking_keeps_later_guard_child_when_current_child_is_empty(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The tracker must prosecute the target and retain the real next child.

    This directly covers the builder that failed in Scenario 112020.  An
    input-level done marker may describe the exhausted current child, but it
    must not erase a later child in the same boundary owner set or replace it
    with a synthetic loiter.
    """

    source_package, source_paths = _guard_remainder(4, 5, (3, 4), token=4)
    current_mission = deepcopy(source_package["individualMissionList"][0])
    follow_up_mission = deepcopy(source_package["individualMissionList"][1])
    current_path = deepcopy(source_paths[0])
    follow_up_path = deepcopy(source_paths[1])
    follow_up_mission["individualMissionID"] = 900_550_003
    follow_up_mission["pathID"] = 400_055_003
    follow_up_path["individualMissionID"] = 900_550_003
    follow_up_path["pathID"] = 400_055_003
    follow_up_path["waypointList"] = [
        _waypoint(550_203, 550_204),
        _waypoint(550_204, 0),
    ]

    monkeypatch.setattr(
        attack,
        "_source_type2_self_reliance_phase",
        lambda **_kwargs: attack.TYPE2_SELF_RELIANCE_GUARD_AREA,
    )
    monkeypatch.setattr(
        attack,
        "_resolve_uav_tracking_flight_altitude",
        lambda **_kwargs: 1000,
    )
    monkeypatch.setattr(
        attack,
        "_split_done_resume_path",
        lambda *_args, **_kwargs: ([], [], 550_200),
    )

    def fail_if_hold_is_built(*_args: Any, **_kwargs: Any) -> None:
        raise AssertionError("pending boundary child must suppress synthetic hold")

    monkeypatch.setattr(
        attack,
        "_build_uav_attack_completion_hold_waypoint",
        fail_if_hold_is_built,
    )
    monkeypatch.setattr(
        attack,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [deepcopy(follow_up_mission)],
            [
                (
                    tmp_path / "FlightPath" / "400055003.json",
                    deepcopy(follow_up_path),
                )
            ],
            {"clonedCount": 1},
        ),
    )
    monkeypatch.setattr(
        attack,
        "_trim_uav_follow_up_paths_after_anchor",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )

    captured_entries: list[tuple[Path, dict[str, Any]]] = []

    def capture_deferred_writes(
        entries: Any,
        *,
        defer_write: bool,
    ) -> tuple[list[dict[str, Any]], list[tuple[Path, dict[str, Any]]]]:
        assert defer_write is True
        rows = [(Path(path), deepcopy(payload)) for path, payload in entries]
        captured_entries.extend(rows)
        return ([{"path": str(path), "written": False} for path, _ in rows], rows)

    monkeypatch.setattr(
        attack,
        "_write_or_defer_attack_json_entries",
        capture_deferred_writes,
    )

    result = attack._build_uav_attack_tracking_package(
        descriptor={
            "aircraft_id": 4,
            "label": "uav_tracker_4",
            "target_id": 13,
            "target_coord": {
                "latitude": 38.001,
                "longitude": 127.001,
                "altitude": 900,
            },
        },
        new_imp_id=800_550_001,
        imp_data=deepcopy(source_package),
        fp_data=current_path,
        target_mission_template=current_mission,
        target_index=0,
        attack_coord={"latitude": 38.001, "longitude": 127.001, "altitude": 900},
        ctx={"mission_ids": [5], "reason": "attack"},
        state={
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
            "heading": 0.0,
            "speed": 40.0,
        },
        artifacts=SimpleNamespace(source_plan_id=700_550_001, path_id=current_path["pathID"]),
        emit=lambda _message: None,
        now_ms=1_785_291_903_000,
        done_input_ids={5},
        id_reservation=_AttackTrackingReservation(),
        defer_write=True,
    )

    assert result is not None
    assert result["completionBoundaryHold"] is False
    assert "resume" not in result
    generated_package = next(
        payload
        for _path, payload in captured_entries
        if "individualMissionPackageID" in payload
    )
    generated_paths = [
        payload for _path, payload in captured_entries if "pathID" in payload
    ]
    assert len(generated_package["individualMissionList"]) == 2
    assert len(generated_paths) == 2
    tracking_mission, guard_mission = generated_package["individualMissionList"]
    assert tracking_mission["individualMissionInfo"]["individualMissionType"] == 1
    assert guard_mission["boundaryGuardSequence"] == 1
    assert guard_mission["boundaryGuardSequenceCount"] == 1
    guard_path = next(path for path in generated_paths if "boundaryGuardSetID" in path)
    assert guard_path["boundaryGuardSequence"] == 1
    assert guard_path["boundaryGuardSequenceCount"] == 1
    assert guard_path["waypointList"][-1]["nextWaypointID"] == 550_203
    assert guard_path["waypointList"][-1]["eta"] == 600
    validate_generated_artifact_payloads(
        individual_mission_plans=[generated_package],
        flight_paths=generated_paths,
        allow_existing_db_artifacts=False,
        scope="attackUavTrackingPendingGuardChildWithoutHold",
    )


class _ReusedGuardRepairReservation:
    def __init__(self) -> None:
        self._imp_id = 800_650_001
        self._path_id = 500_065_000

    def next_imp(self) -> int:
        return int(self._imp_id)

    def next_path(self, aircraft_id: int) -> int:
        assert int(aircraft_id) == 5
        self._path_id += 1
        return int(self._path_id)


def _reused_guard_plan(
    package: dict[str, Any],
) -> dict[str, Any]:
    return {
        "missionPlanID": 700_650_001,
        "aircraftList": [
            {
                "aircraftID": 5,
                "individualMissionPackageID": int(
                    package["individualMissionPackageID"]
                ),
            }
        ],
    }


def _reused_guard_cache(
    package: dict[str, Any],
    paths: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "imp_payloads": {
            int(package["individualMissionPackageID"]): package,
        },
        "fp_payloads": {
            int(path["pathID"]): path
            for path in paths
        },
        "waypoint_ids": {},
    }


def test_attack_reused_valid_boundary_guard_package_is_not_cloned(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    package, paths = _guard_remainder(5, 4, (1, 2, 3, 4), token=6)
    _finalize_guard_rows(package, paths)
    package["individualMissionList"].append(
        {
            "individualMissionID": 900_650_099,
            "isDone": True,
            "individualMissionInfo": {"individualMissionType": 0},
        }
    )
    source_plan = _reused_guard_plan(package)
    candidate_plan = deepcopy(source_plan)
    package_before = deepcopy(package)
    paths_before = deepcopy(paths)

    def fail_reservation(**_kwargs: Any) -> None:
        raise AssertionError("valid reused guard must not reserve IDs")

    monkeypatch.setattr(
        attack.ReplanIdReservation,
        "reserve",
        fail_reservation,
    )
    monkeypatch.setattr(
        attack,
        "get_runtime_float",
        lambda key, default: 600.0
        if key == "type2_boundary_guard_duration_s"
        else default,
    )

    result = attack._repair_reused_boundary_guard_artifacts(
        source_plan_data=source_plan,
        candidate_plan_data=candidate_plan,
        reused_aircraft_ids={5},
        source_artifact_cache=_reused_guard_cache(package, paths),
        now_ms=1_785_291_904_000,
        emit=lambda _message: None,
    )

    assert result["inspectedSetCount"] == 1
    assert result["compatibleSetCount"] == 1
    assert result["repairedSetCount"] == 0
    assert result["write_entries"] == []
    assert candidate_plan == source_plan
    assert package == package_before
    assert paths == paths_before


def test_attack_reused_legacy_eta70_and_path_only_contract_are_cloned_and_repaired(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    legacy_package, legacy_paths = _guard_remainder(
        5,
        4,
        (1, 2, 3, 4),
        token=7,
    )
    valid_package, valid_paths = _guard_remainder(5, 1, (1,), token=8)
    valid_set_id = "type2-boundary:3:6:aircraft-5"
    valid_contract = boundary_guard_contract(
        set_id=valid_set_id,
        sequence=1,
        sequence_count=1,
        duration_s=600,
    )
    apply_boundary_guard_contract(
        valid_package["individualMissionList"][0],
        valid_contract,
        include_individual_mission_info=True,
    )
    apply_boundary_guard_contract(valid_paths[0], valid_contract)
    legacy_package["individualMissionList"].extend(
        deepcopy(valid_package["individualMissionList"])
    )
    all_paths = legacy_paths + valid_paths
    _finalize_guard_rows(legacy_package, all_paths)

    legacy_package["individualMissionList"][0]["isDone"] = True
    legacy_paths[0]["waypointList"][0]["isDone"] = True
    legacy_paths[-1]["waypointList"][-1]["eta"] = 70
    # A legacy one-sided artifact must still be found from its FlightPath.
    path_only_mission = legacy_package["individualMissionList"][1]
    for key in BOUNDARY_GUARD_CONTRACT_KEYS:
        path_only_mission.pop(key, None)
        path_only_mission["individualMissionInfo"].pop(key, None)

    source_plan = _reused_guard_plan(legacy_package)
    candidate_plan = deepcopy(source_plan)
    source_package_before = deepcopy(legacy_package)
    source_paths_before = deepcopy(all_paths)
    requested_settings: list[str] = []

    def runtime_float(key: str, default: float) -> float:
        requested_settings.append(str(key))
        if key == "type2_boundary_guard_duration_s":
            return 600.0
        return float(default)

    monkeypatch.setattr(attack, "get_runtime_float", runtime_float)
    monkeypatch.setattr(
        attack.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )

    result = attack._repair_reused_boundary_guard_artifacts(
        source_plan_data=source_plan,
        candidate_plan_data=candidate_plan,
        reused_aircraft_ids={5},
        source_artifact_cache=_reused_guard_cache(legacy_package, all_paths),
        now_ms=1_785_291_905_000,
        emit=lambda _message: None,
        id_reservation=_ReusedGuardRepairReservation(),
    )

    assert requested_settings == ["type2_boundary_guard_duration_s"]
    assert result["inspectedSetCount"] == 2
    assert result["compatibleSetCount"] == 1
    assert result["repairedSetCount"] == 1
    assert result["repairedAircraftIDs"] == [5]
    assert candidate_plan["aircraftList"][0]["individualMissionPackageID"] == 800_650_001

    entries = list(result["write_entries"])
    cloned_package = next(
        payload
        for _path, payload in entries
        if "individualMissionPackageID" in payload
    )
    cloned_paths = [
        payload
        for _path, payload in entries
        if "pathID" in payload
    ]
    assert len(entries) == 5
    assert len(cloned_paths) == 4
    assert {
        int(path["pathID"]) for path in cloned_paths
    } == {500_065_001, 500_065_002, 500_065_003, 500_065_004}

    source_missions = source_package_before["individualMissionList"]
    cloned_missions = cloned_package["individualMissionList"]
    assert [
        int(mission["individualMissionID"]) for mission in cloned_missions
    ] == [
        int(mission["individualMissionID"]) for mission in source_missions
    ]
    assert [
        bool(mission.get("isDone")) for mission in cloned_missions
    ] == [
        bool(mission.get("isDone")) for mission in source_missions
    ]
    assert int(cloned_missions[-1]["pathID"]) == int(source_missions[-1]["pathID"])

    source_guard_wp_ids = [
        [
            int(waypoint["waypointID"])
            for waypoint in path["waypointList"]
        ]
        for path in legacy_paths
    ]
    cloned_guard_wp_ids = [
        [
            int(waypoint["waypointID"])
            for waypoint in path["waypointList"]
        ]
        for path in cloned_paths
    ]
    assert cloned_guard_wp_ids == source_guard_wp_ids
    assert [
        [
            bool(waypoint.get("isDone"))
            for waypoint in path["waypointList"]
        ]
        for path in cloned_paths
    ] == [
        [
            bool(waypoint.get("isDone"))
            for waypoint in path["waypointList"]
        ]
        for path in legacy_paths
    ]
    assert cloned_paths[-1]["waypointList"][-1]["eta"] == 600
    assert cloned_paths[-1]["waypointList"][-1]["nextWaypointID"] == (
        cloned_paths[0]["waypointList"][0]["waypointID"]
    )
    validate_boundary_guard_flight_path_sets(cloned_paths)
    validate_generated_artifact_payloads(
        individual_mission_plans=[cloned_package],
        flight_paths=cloned_paths + [valid_paths[0]],
        allow_existing_db_artifacts=False,
        scope="attackReusedLegacyBoundaryRepair",
    )

    # The compatibility pass is read-only with respect to every source object.
    assert legacy_package == source_package_before
    assert all_paths == source_paths_before
    assert legacy_paths[-1]["waypointList"][-1]["eta"] == 70
