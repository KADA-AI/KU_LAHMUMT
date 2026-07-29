from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    BOUNDARY_GUARD_CONTRACT_KEYS,
    apply_boundary_guard_contract,
    boundary_guard_contract,
    validate_boundary_guard_flight_path_sets,
)
from modules.mission_planning.replanning.triggers.prior import pipeline as prior


_SET_ID = "type2-boundary:3:5:aircraft-6"


def _waypoint(waypoint_id: int, next_waypoint_id: int = 0) -> dict[str, Any]:
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
        "isDone": False,
    }


def _mission(
    mission_id: int,
    path_id: int,
    *,
    sequence: int,
    include_contract: bool = True,
) -> dict[str, Any]:
    mission = {
        "aircraftID": 6,
        "individualMissionID": int(mission_id),
        "pathID": int(path_id),
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
            "lineList": [],
            "areaList": [],
            "targetID": None,
        },
    }
    if include_contract:
        apply_boundary_guard_contract(
            mission,
            boundary_guard_contract(
                set_id=_SET_ID,
                sequence=int(sequence),
                sequence_count=4,
                duration_s=600,
            ),
            include_individual_mission_info=True,
        )
    return mission


def _path(
    path_id: int,
    mission_id: int,
    first_waypoint_id: int,
    *,
    sequence: int,
    include_contract: bool = True,
) -> dict[str, Any]:
    payload = {
        "pathID": int(path_id),
        "aircraftID": 6,
        "individualMissionID": int(mission_id),
        "waypointList": [
            _waypoint(first_waypoint_id, first_waypoint_id + 1),
            _waypoint(first_waypoint_id + 1),
        ],
    }
    if include_contract:
        apply_boundary_guard_contract(
            payload,
            boundary_guard_contract(
                set_id=_SET_ID,
                sequence=int(sequence),
                sequence_count=4,
                duration_s=600,
            ),
        )
    return payload


class _Reservation:
    def __init__(self, *, new_imp_id: int, resume_path_id: int) -> None:
        self.new_imp_id = int(new_imp_id)
        self.resume_path_id = int(resume_path_id)
        self._waypoint_id = 90_000

    def next_imp(self) -> int:
        return self.new_imp_id

    def next_individual(self) -> int:
        return 900_900_001

    def next_path(self, aircraft_id: int) -> int:
        assert int(aircraft_id) == 6
        return self.resume_path_id

    def next_waypoint(self) -> int:
        self._waypoint_id += 1
        return self._waypoint_id

    def summary(self) -> dict[str, Any]:
        return {"test": "prior-boundary-guard"}


def _drop_contract(payload: dict[str, Any]) -> None:
    for key in BOUNDARY_GUARD_CONTRACT_KEYS:
        payload.pop(key, None)
    info = payload.get("individualMissionInfo")
    if isinstance(info, dict):
        for key in BOUNDARY_GUARD_CONTRACT_KEYS:
            info.pop(key, None)


def _install_common_stubs(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    *,
    source_package: dict[str, Any],
    source_paths: dict[int, dict[str, Any]],
) -> list[tuple[Path, dict[str, Any]]]:
    current_mission = source_package["individualMissionList"][0]
    current_path_id = int(current_mission["pathID"])
    monkeypatch.setattr(
        prior,
        "_resolve_plan_artifacts",
        lambda **_kwargs: SimpleNamespace(
            individual_mission_package_id=int(
                source_package["individualMissionPackageID"]
            ),
            individual_mission_id=int(current_mission["individualMissionID"]),
            path_id=current_path_id,
        ),
    )
    monkeypatch.setattr(
        prior.db_paths,
        "get_db_subpath",
        lambda *parts: tmp_path.joinpath(*parts),
    )

    def read_payload(path: Any, **_kwargs: Any) -> dict[str, Any]:
        path_obj = Path(path)
        if path_obj.parent.name == "IndividualMissionPlan":
            return deepcopy(source_package)
        return deepcopy(source_paths[int(path_obj.stem)])

    monkeypatch.setattr(prior, "read_json_cached", read_payload)
    monkeypatch.setattr(
        prior,
        "_source_input_mission_is_locked_type2_branch",
        lambda *_args, **_kwargs: False,
    )
    monkeypatch.setattr(
        prior,
        "_sync_resume_mission_info_with_waypoints",
        lambda *_args, **_kwargs: False,
    )

    captured: list[tuple[Path, dict[str, Any]]] = []

    def capture_write(entries: Any, **_kwargs: Any) -> list[dict[str, Any]]:
        rows = [(Path(path), deepcopy(payload)) for path, payload in entries]
        captured.extend(rows)
        return [{"path": str(path), "written": True} for path, _payload in rows]

    monkeypatch.setattr(prior, "write_json_batch", capture_write)
    return captured


def test_general_prior_forces_guard_followup_clone_and_finalizes_remainder(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """clone_follow_up_artifacts=False must still rebuild a guard owner set."""

    current_mission = _mission(900_001_001, 600_001_001, sequence=3)
    follow_up_source_mission = _mission(900_001_002, 600_001_002, sequence=4)
    # Reproduce a legacy mission-only contract on both source paths.
    current_source_path = _path(
        600_001_001,
        900_001_001,
        10_001,
        sequence=3,
        include_contract=False,
    )
    follow_up_source_path = _path(
        600_001_002,
        900_001_002,
        10_003,
        sequence=4,
        include_contract=False,
    )
    source_package = {
        "individualMissionPackageID": 800_001_001,
        "aircraftID": 6,
        "individualMissionList": [current_mission, follow_up_source_mission],
    }
    captured = _install_common_stubs(
        monkeypatch,
        tmp_path,
        source_package=source_package,
        source_paths={
            600_001_001: current_source_path,
            600_001_002: follow_up_source_path,
        },
    )
    monkeypatch.setattr(prior, "_load_done_input_ids_for_plan", lambda _plan_id: set())
    monkeypatch.setattr(
        prior,
        "_apply_resume_path_trimming",
        lambda *_args, **_kwargs: (
            [],
            [_waypoint(20_001, 20_002), _waypoint(20_002)],
            10_000,
        ),
    )

    clone_calls: list[dict[str, Any]] = []

    def clone_followups(**kwargs: Any) -> tuple[list[dict[str, Any]], list[tuple[Path, dict[str, Any]]]]:
        clone_calls.append(kwargs)
        mission = deepcopy(kwargs["missions"][0])
        mission["individualMissionID"] = 900_001_102
        mission["pathID"] = 600_001_102
        path = _path(
            600_001_102,
            900_001_102,
            20_003,
            sequence=4,
            include_contract=False,
        )
        return [mission], [(tmp_path / "FlightPath" / "600001102.json", path)]

    monkeypatch.setattr(prior, "_clone_follow_up_replan_artifacts", clone_followups)

    result = prior._build_other_uav_resume_package(
        source_plan_id=700_001_001,
        aircraft_id=6,
        current_waypoint_id=10_001,
        current_coord={"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
        emit=lambda _message: None,
        now_ms=1_785_292_000_000,
        sweep_progress=None,
        clone_follow_up_artifacts=False,
        include_done_reference_mission=False,
        id_reservation=_Reservation(
            new_imp_id=800_001_101,
            resume_path_id=600_001_101,
        ),
    )

    assert result is not None
    assert len(clone_calls) == 1
    package = next(
        payload for _path, payload in captured if "individualMissionPackageID" in payload
    )
    paths = [payload for _path, payload in captured if "pathID" in payload]
    assert [mission["boundaryGuardSequence"] for mission in package["individualMissionList"]] == [1, 2]
    assert {mission["boundaryGuardSequenceCount"] for mission in package["individualMissionList"]} == {2}
    assert [path["boundaryGuardSequence"] for path in paths] == [1, 2]
    assert paths[0]["waypointList"][-1]["nextWaypointID"] == 20_003
    assert paths[1]["waypointList"][-1]["nextWaypointID"] == 20_001
    assert paths[1]["waypointList"][-1]["eta"] == 600
    validate_boundary_guard_flight_path_sets(paths)


def test_prior_keeps_pending_guard_suffix_when_current_child_is_empty(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A stale input-done bit cannot turn a pending AREA child into no update."""

    current_mission = _mission(900_002_001, 600_002_001, sequence=3)
    # Reproduce the inverse legacy asymmetry: only the source FlightPath has
    # the later child's contract.  Pending-child detection must still find it.
    follow_up_source_mission = _mission(
        900_002_002,
        600_002_002,
        sequence=4,
        include_contract=False,
    )
    current_source_path = _path(600_002_001, 900_002_001, 30_001, sequence=3)
    follow_up_source_path = _path(600_002_002, 900_002_002, 30_003, sequence=4)
    source_package = {
        "individualMissionPackageID": 800_002_001,
        "aircraftID": 6,
        "individualMissionList": [current_mission, follow_up_source_mission],
    }
    captured = _install_common_stubs(
        monkeypatch,
        tmp_path,
        source_package=source_package,
        source_paths={
            600_002_001: current_source_path,
            600_002_002: follow_up_source_path,
        },
    )
    monkeypatch.setattr(prior, "_load_done_input_ids_for_plan", lambda _plan_id: {5})
    monkeypatch.setattr(
        prior,
        "_apply_resume_path_trimming",
        lambda *_args, **_kwargs: (
            [_waypoint(30_001, 30_002), _waypoint(30_002)],
            [],
            30_002,
        ),
    )

    clone_calls: list[dict[str, Any]] = []

    def clone_followups(**kwargs: Any) -> tuple[list[dict[str, Any]], list[tuple[Path, dict[str, Any]]]]:
        clone_calls.append(kwargs)
        # The current input must have been removed from aggregate done
        # filtering because this concrete later child is still pending.
        assert 5 not in set(kwargs["excluded_input_ids"])
        assert kwargs["missions"][0]["boundaryGuardSetID"] == _SET_ID
        mission = deepcopy(kwargs["missions"][0])
        mission["individualMissionID"] = 900_002_102
        mission["pathID"] = 600_002_102
        _drop_contract(mission)
        path = _path(600_002_102, 900_002_102, 40_003, sequence=4)
        return [mission], [(tmp_path / "FlightPath" / "600002102.json", path)]

    monkeypatch.setattr(prior, "_clone_follow_up_replan_artifacts", clone_followups)

    result = prior._build_other_uav_resume_package(
        source_plan_id=700_002_001,
        aircraft_id=6,
        current_waypoint_id=30_001,
        current_coord={"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
        emit=lambda _message: None,
        now_ms=1_785_292_001_000,
        sweep_progress=None,
        clone_follow_up_artifacts=False,
        include_done_reference_mission=False,
        id_reservation=_Reservation(
            new_imp_id=800_002_101,
            resume_path_id=600_002_101,
        ),
    )

    assert result is not None
    assert len(clone_calls) == 1
    assert result["boundaryGuardCurrentChildOmitted"] is True
    assert result["resumePath"] is None
    assert "resume" not in result
    package = next(
        payload for _path, payload in captured if "individualMissionPackageID" in payload
    )
    paths = [payload for _path, payload in captured if "pathID" in payload]
    assert len(package["individualMissionList"]) == 1
    assert len(paths) == 1
    mission = package["individualMissionList"][0]
    path = paths[0]
    assert mission["boundaryGuardSequence"] == 1
    assert mission["boundaryGuardSequenceCount"] == 1
    assert path["boundaryGuardSequence"] == 1
    assert path["boundaryGuardSequenceCount"] == 1
    assert path["waypointList"][-1]["nextWaypointID"] == 40_003
    assert path["waypointList"][-1]["eta"] == 600
    validate_boundary_guard_flight_path_sets(paths)
