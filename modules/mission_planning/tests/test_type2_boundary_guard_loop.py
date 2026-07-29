from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0302,
)
from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    BOUNDARY_GUARD_CONTRACT_KEYS,
    annotate_boundary_guard_set,
    apply_boundary_guard_contract,
    boundary_guard_contract,
    finalize_boundary_guard_flight_path_sets_in_mission_order,
    link_boundary_guard_flight_path_sets,
    resequence_boundary_guard_flight_path_sets,
    validate_boundary_guard_flight_path_sets,
)
from modules.mission_planning.runtime import aircraft_parallel_0303
from modules.mission_planning.runtime.validation.replan_payloads import (
    ReplanValidationError,
    validate_generated_artifact_payloads,
)


def _waypoint(waypoint_id: int, next_waypoint_id: int, eta: int) -> dict:
    return {
        "waypointID": int(waypoint_id),
        "coordinate": {
            "latitude": 38.0 + waypoint_id * 1e-7,
            "longitude": 127.0,
            "altitude": 1000,
        },
        "speed": 40.0,
        "eta": int(eta),
        "ecf": 0.0,
        "nextWaypointID": int(next_waypoint_id),
        "waypointPassType": 1,
    }


def _guard_paths() -> list[dict]:
    paths = [
        {
            "pathID": 400_000_001,
            "aircraftID": 4,
            "waypointList": [
                _waypoint(101, 102, 0),
                _waypoint(102, 0, 11_000),
            ],
        },
        {
            "pathID": 400_000_002,
            "aircraftID": 4,
            "waypointList": [
                _waypoint(201, 202, 0),
                _waypoint(202, 0, 13_000),
            ],
        },
    ]
    annotate_boundary_guard_set(
        paths,
        set_id="type2-boundary:200000201:202:aircraft-4",
        duration_s=37,
    )
    return paths


def test_boundary_guard_links_full_owner_set_and_sets_only_cycle_tail_eta() -> None:
    paths = _guard_paths()
    eta_before = [
        [waypoint["eta"] for waypoint in path["waypointList"]]
        for path in paths
    ]

    summary = link_boundary_guard_flight_path_sets(paths)

    assert paths[0]["waypointList"][-1]["nextWaypointID"] == 201
    assert paths[1]["waypointList"][-1]["nextWaypointID"] == 101
    assert {
        path["boundaryGuardCycleFirstWaypointID"] for path in paths
    } == {101}
    assert {
        path["boundaryGuardCycleLastWaypointID"] for path in paths
    } == {202}
    assert [waypoint["eta"] for waypoint in paths[0]["waypointList"]] == eta_before[0]
    assert paths[1]["waypointList"][0]["eta"] == eta_before[1][0]
    assert paths[1]["waypointList"][-1]["eta"] == 37
    assert summary[paths[0]["boundaryGuardSetID"]]["sequenceCount"] == 2
    validate_boundary_guard_flight_path_sets(paths)


def test_boundary_guard_rejects_cross_aircraft_or_wrong_declared_tail() -> None:
    paths = _guard_paths()
    paths[1]["aircraftID"] = 5
    with pytest.raises(ValueError, match="same positive aircraftID"):
        link_boundary_guard_flight_path_sets(paths)

    paths = _guard_paths()
    link_boundary_guard_flight_path_sets(paths)
    paths[0]["waypointList"][-1]["nextWaypointID"] = 202
    with pytest.raises(ValueError, match="expected 201"):
        validate_boundary_guard_flight_path_sets(paths)


def test_parallel_waypoint_reassignment_restores_the_complete_guard_cycle() -> None:
    paths = _guard_paths()
    link_boundary_guard_flight_path_sets(paths)
    missions = [
        {
            "pathID": path["pathID"],
            "individualMissionInfo": {},
        }
        for path in paths
    ]

    class Allocator:
        def __init__(self) -> None:
            self.value = 70

        def alloc(self) -> int:
            value = self.value
            self.value += 1
            return value

    aircraft_parallel_0303.reassign_waypoint_ids_inplace(
        paths,
        Allocator(),
        missions=missions,
    )

    assert [
        waypoint["waypointID"]
        for path in paths
        for waypoint in path["waypointList"]
    ] == [70, 71, 72, 73]
    assert paths[0]["waypointList"][-1]["nextWaypointID"] == 72
    assert paths[1]["waypointList"][-1]["nextWaypointID"] == 70
    assert {path["boundaryGuardCycleFirstWaypointID"] for path in paths} == {70}
    assert {path["boundaryGuardCycleLastWaypointID"] for path in paths} == {73}
    assert {mission["boundaryGuardCycleFirstWaypointID"] for mission in missions} == {70}
    assert {
        mission["individualMissionInfo"]["boundaryGuardCycleLastWaypointID"]
        for mission in missions
    } == {73}
    assert aircraft_parallel_0303.validate_0303_parallel_merge_output(paths)["valid"] is True

    paths[0]["waypointList"][-1]["nextWaypointID"] = 0
    validation = aircraft_parallel_0303.validate_0303_parallel_merge_output(paths)
    assert validation["valid"] is False
    assert any(
        str(error).startswith("boundaryGuardLoop:")
        for error in validation["errors"]
    )


def test_resume_finalizer_uses_imp_order_for_split_current_and_cloned_children() -> None:
    set_id = "type2-boundary:3:5:aircraft-4"
    resume = {
        "pathID": 400_000_101,
        "aircraftID": 4,
        "waypointList": [
            _waypoint(301, 302, 0),
            _waypoint(302, 0, 7_000),
        ],
    }
    apply_boundary_guard_contract(
        resume,
        boundary_guard_contract(
            set_id=set_id,
            sequence=3,
            sequence_count=5,
        ),
    )
    cloned_follow_ups = [
        {
            "pathID": 400_000_102,
            "aircraftID": 4,
            "waypointList": [
                _waypoint(401, 402, 0),
                _waypoint(402, 0, 8_000),
            ],
        },
        {
            "pathID": 400_000_103,
            "aircraftID": 4,
            "waypointList": [
                _waypoint(501, 502, 0),
                _waypoint(502, 0, 9_000),
            ],
        },
    ]
    # This is the intermediate state from the clone helper: it only saw the
    # later two children and therefore numbered that subset as 1..2.
    annotate_boundary_guard_set(
        cloned_follow_ups,
        set_id=set_id,
    )
    paths = [resume, *cloned_follow_ups]
    missions = []
    for index, path in enumerate(paths, start=1):
        mission = {
            "individualMissionID": 900_001_000 + index,
            "pathID": path["pathID"],
            "individualMissionInfo": {},
        }
        apply_boundary_guard_contract(
            mission,
            path,
            include_individual_mission_info=True,
        )
        missions.append(mission)

    summary = finalize_boundary_guard_flight_path_sets_in_mission_order(
        missions,
        paths,
    )

    assert [path["boundaryGuardSequence"] for path in paths] == [1, 2, 3]
    assert {path["boundaryGuardSequenceCount"] for path in paths} == {3}
    assert [mission["boundaryGuardSequence"] for mission in missions] == [1, 2, 3]
    assert {mission["boundaryGuardSequenceCount"] for mission in missions} == {3}
    assert resume["waypointList"][-1]["nextWaypointID"] == 401
    assert cloned_follow_ups[0]["waypointList"][-1]["nextWaypointID"] == 501
    assert cloned_follow_ups[1]["waypointList"][-1]["nextWaypointID"] == 301
    assert summary[set_id]["sequenceCount"] == 3
    validate_boundary_guard_flight_path_sets(paths)


@pytest.mark.parametrize("contract_side", ["mission", "path"])
def test_resume_finalizer_repairs_one_sided_legacy_guard_contract(
    contract_side: str,
) -> None:
    """Historical IMP/path pairs may carry the duplicated contract on one side."""

    paths = _guard_paths()
    missions = []
    for index, path in enumerate(paths, start=1):
        mission = {
            "aircraftID": 4,
            "individualMissionID": 900_001_050 + index,
            "pathID": path["pathID"],
            "individualMissionInfo": {},
        }
        if contract_side == "mission":
            apply_boundary_guard_contract(
                mission,
                path,
                include_individual_mission_info=True,
            )
            for key in BOUNDARY_GUARD_CONTRACT_KEYS:
                path.pop(key, None)
        missions.append(mission)

    summary = finalize_boundary_guard_flight_path_sets_in_mission_order(
        missions,
        paths,
    )

    set_id = "type2-boundary:200000201:202:aircraft-4"
    assert summary[set_id]["sequenceCount"] == 2
    assert [mission["boundaryGuardSequence"] for mission in missions] == [1, 2]
    assert [path["boundaryGuardSequence"] for path in paths] == [1, 2]
    assert paths[0]["waypointList"][-1]["nextWaypointID"] == 201
    assert paths[1]["waypointList"][-1]["nextWaypointID"] == 101
    validate_boundary_guard_flight_path_sets(paths)


def test_reexecute_finalizer_repairs_stale_cross_path_links_after_independent_remap() -> None:
    """Current-mission refresh remaps carried paths one path at a time.

    Internal next IDs can be remapped locally, but a guard tail points into the
    next path and therefore remains at its old source ID until the store-time
    guard finalizer rebuilds the complete cycle.
    """

    paths = _guard_paths()
    link_boundary_guard_flight_path_sets(paths)
    missions = []
    for index, path in enumerate(paths, start=1):
        mission = {
            "individualMissionID": 900_001_100 + index,
            "pathID": path["pathID"],
            "individualMissionInfo": {},
        }
        apply_boundary_guard_contract(
            mission,
            path,
            include_individual_mission_info=True,
        )
        missions.append(mission)

    first_rows = paths[0]["waypointList"]
    second_rows = paths[1]["waypointList"]
    first_rows[0]["waypointID"] = 10_182
    first_rows[0]["nextWaypointID"] = 10_183
    first_rows[1]["waypointID"] = 10_183
    # Old cross-path source ID survives the per-path remap.
    assert first_rows[1]["nextWaypointID"] == 201
    second_rows[0]["waypointID"] = 10_186
    second_rows[0]["nextWaypointID"] = 10_187
    second_rows[1]["waypointID"] = 10_187
    assert second_rows[1]["nextWaypointID"] == 101

    with pytest.raises(ValueError, match="expected 10186"):
        validate_boundary_guard_flight_path_sets(paths)

    summary = finalize_boundary_guard_flight_path_sets_in_mission_order(
        missions,
        paths,
    )

    assert first_rows[-1]["nextWaypointID"] == 10_186
    assert second_rows[-1]["nextWaypointID"] == 10_182
    assert {path["boundaryGuardCycleFirstWaypointID"] for path in paths} == {10_182}
    assert {mission["boundaryGuardCycleFirstWaypointID"] for mission in missions} == {10_182}
    assert summary[paths[0]["boundaryGuardSetID"]]["cycleFirstWaypointID"] == 10_182
    validate_boundary_guard_flight_path_sets(paths)


def test_attack_return_resequences_remaining_children_and_restores_the_cycle() -> None:
    paths = _guard_paths()
    third = {
        "pathID": 400_000_003,
        "aircraftID": 4,
        "waypointList": [
            _waypoint(301, 302, 0),
            _waypoint(302, 0, 7_000),
        ],
    }
    paths.append(third)
    annotate_boundary_guard_set(
        paths,
        set_id="type2-boundary:200000201:202:aircraft-4",
        duration_s=600,
    )

    # The first child was completed before tracking/attack.  The return replan
    # keeps only the two unfinished children and its normal reset makes both
    # tails terminal until the common guard post-pass restores the cycle.
    remaining = deepcopy(paths[1:])
    for path in remaining:
        path["waypointList"][-1]["nextWaypointID"] = 0

    resequence_boundary_guard_flight_path_sets(remaining)
    link_boundary_guard_flight_path_sets(remaining)

    assert [path["boundaryGuardSequence"] for path in remaining] == [1, 2]
    assert [path["boundaryGuardSequenceCount"] for path in remaining] == [2, 2]
    assert remaining[0]["waypointList"][-1]["nextWaypointID"] == 301
    assert remaining[1]["waypointList"][-1]["nextWaypointID"] == 201
    validate_boundary_guard_flight_path_sets(remaining)


def _mission(
    *,
    mission_id: int,
    path_id: int,
    contract: dict,
) -> dict:
    row = {
        "aircraftID": 4,
        "individualMissionID": int(mission_id),
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 1,
            "inputMissionID": 202,
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
    apply_boundary_guard_contract(
        row,
        contract,
        include_individual_mission_info=True,
    )
    return row


def test_generated_artifact_validator_allows_only_declared_guard_cross_path() -> None:
    paths = _guard_paths()
    link_boundary_guard_flight_path_sets(paths)
    for mission_id, path in zip((900_000_001, 900_000_002), paths):
        path["individualMissionID"] = mission_id
    missions = [
        _mission(
            mission_id=mission_id,
            path_id=path["pathID"],
            contract=path,
        )
        for mission_id, path in zip((900_000_001, 900_000_002), paths)
    ]
    package = {
        "individualMissionPackageID": 800_000_001,
        "aircraftID": 4,
        "individualMissionList": missions,
    }

    summary = validate_generated_artifact_payloads(
        individual_mission_plans=[package],
        flight_paths=paths,
        allow_existing_db_artifacts=False,
        scope="boundaryGuardTest",
    )
    assert summary["flightPaths"] == 2

    invalid = deepcopy(paths)
    invalid[0]["waypointList"][-1]["nextWaypointID"] = 202
    with pytest.raises(ReplanValidationError, match="expected 201"):
        validate_generated_artifact_payloads(
            individual_mission_plans=[package],
            flight_paths=invalid,
            allow_existing_db_artifacts=False,
            scope="boundaryGuardTest",
        )


def test_d0302_preserves_boundary_contract_top_level_and_in_info() -> None:
    pre_contract = boundary_guard_contract(
        set_id="type2-boundary:200000201:202:aircraft-4",
        sequence=1,
        sequence_count=1,
        duration_s=600,
    )
    contract = {
        **pre_contract,
        "boundaryGuardCycleFirstWaypointID": 101,
        "boundaryGuardCycleLastWaypointID": 202,
    }
    raw_mission = _mission(
        mission_id=900_000_001,
        path_id=400_000_010,
        contract=contract,
    )

    packages = d0302.build_mission_packages(
        [raw_mission],
        cmpk_id=200_000_201,
        plan_pkg_map={4: 800_000_001},
        reserved_individual_mission_ids=[900_000_101],
    )
    exported = packages[0]["individualMissionList"][0]
    info = exported["individualMissionInfo"]
    for key in BOUNDARY_GUARD_CONTRACT_KEYS:
        assert exported[key] == contract[key]
        assert info[key] == contract[key]


def test_ordinary_flight_path_is_not_turned_into_a_loop() -> None:
    ordinary = {
        "pathID": 400_000_003,
        "aircraftID": 4,
        "waypointList": [_waypoint(301, 0, 0)],
    }
    link_boundary_guard_flight_path_sets([ordinary])
    assert ordinary["waypointList"][-1]["nextWaypointID"] == 0
    assert "boundaryGuardCycleFirstWaypointID" not in ordinary


def test_real_type2_split_export_and_d0303_link_every_owner_child(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        run_split_pipeline,
    )
    from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (
        build_0302_packages_from_split_with_lah,
    )
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )
    from modules.mission_planning.runtime.state import branch_ownership
    from modules.mission_planning.tests.test_type2_branch_ownership import (
        BRANCH_MISSION_IDS,
        UAV_IDS,
        _mission_reference,
        _type2_branch_package,
    )

    monkeypatch.setattr(
        branch_ownership,
        "_state_path",
        lambda: tmp_path / "branch_ownership.json",
    )
    input_package = _type2_branch_package()
    split_result = run_split_pipeline(
        deepcopy(input_package),
        deepcopy(_mission_reference()),
        list(UAV_IDS),
        apply_assignment=True,
        apply_scheduling=False,
    )
    guard_pieces = [
        piece
        for piece in split_result.pieces
        if int(piece.mission_id) == int(BRANCH_MISSION_IDS[1])
    ]
    assert len(guard_pieces) == 6
    assert all(
        piece.data.get("_type2BoundaryGuardArea") is True
        for piece in guard_pieces
    )

    packages = build_0302_packages_from_split_with_lah(
        split_result,
        cmpk=deepcopy(input_package),
    )
    missions: list[dict] = []
    for package in packages:
        aircraft_id = int(package["aircraftID"])
        if aircraft_id < 4:
            continue
        for mission in package["individualMissionList"]:
            if (
                int(mission["relatedMission"]["inputMissionID"])
                != int(BRANCH_MISSION_IDS[1])
            ):
                continue
            copied = dict(mission)
            copied["aircraftID"] = aircraft_id
            missions.append(copied)

    assert len(missions) == 6
    for aircraft_id in UAV_IDS:
        owner_rows = [
            mission for mission in missions
            if int(mission["aircraftID"]) == int(aircraft_id)
        ]
        assert [row["boundaryGuardSequence"] for row in owner_rows] == [1, 2]
        assert {row["boundaryGuardSequenceCount"] for row in owner_rows} == {2}
        assert {row["boundaryGuardDurationS"] for row in owner_rows} == {600.0}

    parallel_missions = deepcopy(missions)
    flight_paths = d0303.build_flight_plans(
        missions,
        wp_alloc=d0303._WPAllocator(start=1_000),
        cruise_speed=40.0,
        turn_step_deg=15.0,
    )
    assert len(flight_paths) == 6
    for aircraft_id in UAV_IDS:
        owner_paths = sorted(
            [
                path for path in flight_paths
                if int(path["aircraftID"]) == int(aircraft_id)
            ],
            key=lambda path: int(path["boundaryGuardSequence"]),
        )
        assert len(owner_paths) == 2
        first_waypoint_id = owner_paths[0]["waypointList"][0]["waypointID"]
        second_waypoint_id = owner_paths[1]["waypointList"][0]["waypointID"]
        assert (
            owner_paths[0]["waypointList"][-1]["nextWaypointID"]
            == second_waypoint_id
        )
        assert (
            owner_paths[1]["waypointList"][-1]["nextWaypointID"]
            == first_waypoint_id
        )
        validate_boundary_guard_flight_path_sets(owner_paths)

    parallel_result = aircraft_parallel_0303.build_0303_flight_plans_aircraft_parallel(
        d0303,
        parallel_missions,
        runtime_payload={
            "values": {
                "replan_0303_aircraft_workers": 3,
                "replan_0303_aircraft_process_parallel_enabled": False,
            }
        },
        wp_alloc=d0303._WPAllocator(start=2_000),
        cruise_speed=40.0,
        turn_step_deg=15.0,
        ref0203=None,
        env={
            "REPLAN_0303_AIRCRAFT_PARALLEL": "1",
            "REPLAN_0303_AIRCRAFT_PROCESS_PARALLEL": "0",
            "REPLAN_0303_AIRCRAFT_WORKERS": "3",
        },
    )
    assert parallel_result["mode"] == "aircraft_parallel"
    parallel_paths = parallel_result["plans"]
    assert len(parallel_paths) == 6
    mission_by_path_id = {
        int(mission["pathID"]): mission
        for mission in parallel_missions
    }
    for aircraft_id in UAV_IDS:
        owner_paths = sorted(
            [
                path for path in parallel_paths
                if int(path["aircraftID"]) == int(aircraft_id)
            ],
            key=lambda path: int(path["boundaryGuardSequence"]),
        )
        validate_boundary_guard_flight_path_sets(owner_paths)
        first_id = int(owner_paths[0]["waypointList"][0]["waypointID"])
        last_id = int(owner_paths[-1]["waypointList"][-1]["waypointID"])
        for path in owner_paths:
            mission = mission_by_path_id[int(path["pathID"])]
            assert path["boundaryGuardCycleFirstWaypointID"] == first_id
            assert path["boundaryGuardCycleLastWaypointID"] == last_id
            assert mission["boundaryGuardCycleFirstWaypointID"] == first_id
            assert (
                mission["individualMissionInfo"]["boundaryGuardCycleLastWaypointID"]
                == last_id
            )


def test_type3_area_never_receives_the_type2_boundary_loop_marker(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        run_split_pipeline,
    )
    from modules.mission_planning.runtime.state import branch_ownership
    from modules.mission_planning.tests.test_type2_branch_ownership import (
        _mission_reference,
        _type2_branch_package,
    )

    monkeypatch.setattr(
        branch_ownership,
        "_state_path",
        lambda: tmp_path / "branch_ownership.json",
    )
    split_result = run_split_pipeline(
        deepcopy(_type2_branch_package(package_type=3)),
        deepcopy(_mission_reference()),
        [4, 5, 6],
        apply_assignment=True,
        apply_scheduling=False,
    )
    assert not any(
        piece.data.get("_type2BoundaryGuardArea") is True
        for piece in split_result.pieces
    )


def test_next_collab_finalizer_rebuilds_the_strict_guard_owner_set(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )
    from modules.mission_planning.tests.test_type2_branch_ownership import (
        BRANCH_MISSION_IDS,
        _type2_branch_package,
    )

    missions = [
        {
            "aircraftID": 4,
            "individualMissionID": 900_000_001 + index,
            "isDone": False,
            "relatedMission": {
                "relatedMissionType": 1,
                "inputMissionID": int(BRANCH_MISSION_IDS[1]),
                "priorMissionID": 0,
            },
            "individualMissionInfo": {
                "individualMissionType": 3,
                "patternType": 0,
                "areaList": [],
            },
            "pathID": 400_000_001 + index,
        }
        for index in range(2)
    ]
    paths = {
        400_000_001: {
            "pathID": 400_000_001,
            "aircraftID": 4,
            "waypointList": [_waypoint(101, 0, 1_000)],
        },
        400_000_002: {
            "pathID": 400_000_002,
            "aircraftID": 4,
            "waypointList": [_waypoint(201, 0, 2_000)],
        },
    }
    prepared = next_collab_pipeline._PreparedReplacements(
        replacement_by_aircraft={4: missions},
        generated_fp_by_path=paths,
        generated_path_ids=set(paths),
        planner_workflow="test",
        planner_result_text="",
        planned_result_count=2,
        review_report={},
        mission_mode="area",
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "get_runtime_float",
        lambda key, default: 42.0
        if key == "type2_boundary_guard_duration_s"
        else default,
    )

    finalized = next_collab_pipeline._apply_type2_boundary_guard_loop_to_prepared(
        prepared,
        input_data=_type2_branch_package(),
        input_package_id=200_000_201,
        target_input_id=int(BRANCH_MISSION_IDS[1]),
    )

    assert [row["boundaryGuardSequence"] for row in missions] == [1, 2]
    assert {row["boundaryGuardDurationS"] for row in missions} == {42.0}
    assert paths[400_000_001]["waypointList"][-1]["nextWaypointID"] == 201
    assert paths[400_000_002]["waypointList"][-1]["nextWaypointID"] == 101
    assert paths[400_000_001]["waypointList"][-1]["eta"] == 1_000
    assert paths[400_000_002]["waypointList"][-1]["eta"] == 42
    assert finalized.review_report["boundaryGuardLoop"]["enabled"] is True


def test_next_collab_finalizer_repairs_guard_count_after_reexecute_clones(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Repeated current-mission clones must not preserve a stale child count."""

    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        resolve_type2_self_reliance_phase,
    )
    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )
    from modules.mission_planning.tests.test_type2_branch_ownership import (
        BRANCH_MISSION_IDS,
        PACKAGE_ID,
        _type2_branch_package,
    )

    input_package = _type2_branch_package()
    outbound = input_package["inputMissionList"][0]
    guard_area = input_package["inputMissionList"][1]
    reexecute_one = deepcopy(outbound)
    reexecute_one["inputMissionID"] = 209
    reexecute_one["isDone"] = True
    reexecute_two = deepcopy(outbound)
    reexecute_two["inputMissionID"] = 210
    reexecute_two["isDone"] = True
    input_package["inputMissionList"][1:1] = [reexecute_one, reexecute_two]

    # This is the recorded failure shape: synthetic LINE clones make the
    # package-level three-phase resolver intentionally return None even though
    # the prepared target paths still carry the authoritative guard marker.
    assert (
        resolve_type2_self_reliance_phase(
            input_package,
            BRANCH_MISSION_IDS[1],
        )
        is None
    )
    assert guard_area is input_package["inputMissionList"][3]

    stale_set_id = (
        f"type2-boundary:{PACKAGE_ID}:{BRANCH_MISSION_IDS[1]}:aircraft-5"
    )
    missions: list[dict] = []
    paths: dict[int, dict] = {}
    for index in range(4):
        path_id = 500_000_101 + index
        mission = {
            "aircraftID": 5,
            "individualMissionID": 900_000_101 + index,
            "isDone": False,
            "relatedMission": {
                "relatedMissionType": 1,
                "inputMissionID": int(BRANCH_MISSION_IDS[1]),
                "priorMissionID": 0,
            },
            "individualMissionInfo": {
                "individualMissionType": 3,
                "patternType": 0,
                "areaList": [],
            },
            "pathID": path_id,
        }
        path = {
            "pathID": path_id,
            "aircraftID": 5,
            "individualMissionID": mission["individualMissionID"],
            "waypointList": [_waypoint(301 + index, 0, 1_000 + index)],
        }
        stale_contract = boundary_guard_contract(
            set_id=stale_set_id,
            sequence=index + 1,
            sequence_count=5,
            duration_s=600,
        )
        apply_boundary_guard_contract(
            mission,
            stale_contract,
            include_individual_mission_info=True,
        )
        apply_boundary_guard_contract(path, stale_contract)
        missions.append(mission)
        paths[path_id] = path

    prepared = next_collab_pipeline._PreparedReplacements(
        replacement_by_aircraft={5: missions},
        generated_fp_by_path=paths,
        generated_path_ids=set(paths),
        planner_workflow="reexecute-regression",
        planner_result_text="",
        planned_result_count=4,
        review_report={},
        mission_mode="area",
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "get_runtime_float",
        lambda key, default: 600.0
        if key == "type2_boundary_guard_duration_s"
        else default,
    )

    finalized = next_collab_pipeline._apply_type2_boundary_guard_loop_to_prepared(
        prepared,
        input_data=input_package,
        input_package_id=PACKAGE_ID,
        target_input_id=int(BRANCH_MISSION_IDS[1]),
    )

    assert [path["boundaryGuardSequence"] for path in paths.values()] == [1, 2, 3, 4]
    assert {path["boundaryGuardSequenceCount"] for path in paths.values()} == {4}
    assert {mission["boundaryGuardSequenceCount"] for mission in missions} == {4}
    assert paths[500_000_101]["waypointList"][-1]["nextWaypointID"] == 302
    assert paths[500_000_104]["waypointList"][-1]["nextWaypointID"] == 301
    assert (
        finalized.review_report["boundaryGuardLoop"]["detectionSource"]
        == "inherited_guard_contract"
    )
    validate_boundary_guard_flight_path_sets(paths.values())
