from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
    run_split_pipeline,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    SplitRunResult,
)
from modules.mission_planning.runtime.state import branch_ownership


PACKAGE_ID = 200_000_201
BRANCH_MISSION_IDS = (201, 202, 203)
UAV_IDS = [4, 5, 6]
SINGLE_BRANCH_MISSION_IDS = (301, 302, 303)


def _coord(latitude: float, longitude: float) -> dict[str, float]:
    return {
        "latitude": float(latitude),
        "longitude": float(longitude),
        "altitude": 700.0,
    }


def _line_branch(longitude: float, start_latitude: float, end_latitude: float) -> dict[str, Any]:
    return {
        "width": 900.0,
        "coordinateList": [
            _coord(start_latitude, longitude),
            _coord(end_latitude, longitude),
        ],
    }


def _area_branch(longitude: float) -> dict[str, Any]:
    half_width = 0.004
    return {
        "isHole": False,
        "coordinateList": [
            _coord(38.010, longitude - half_width),
            _coord(38.010, longitude + half_width),
            _coord(38.020, longitude + half_width),
            _coord(38.020, longitude - half_width),
        ],
    }


def _type2_branch_package(*, package_type: int = 2) -> dict[str, Any]:
    """Small Type-2 LINE -> AREA -> return LINE fixture with two branches."""

    branch_longitudes = (127.000, 127.100)
    return {
        "inputMissionPackageID": PACKAGE_ID,
        "inputMissionPackageType": int(package_type),
        "availableAircraftList": [{"aircraftID": aircraft_id} for aircraft_id in UAV_IDS],
        "inputMissionList": [
            {
                "inputMissionID": BRANCH_MISSION_IDS[0],
                "inputMissionType": 1,
                "regionType": 7,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [
                        _line_branch(longitude, 38.000, 38.010)
                        for longitude in branch_longitudes
                    ],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": BRANCH_MISSION_IDS[1],
                "inputMissionType": 3,
                "regionType": 7,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [
                        _area_branch(longitude) for longitude in branch_longitudes
                    ],
                },
            },
            {
                "inputMissionID": BRANCH_MISSION_IDS[2],
                "inputMissionType": 1,
                "regionType": 6,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [
                        _line_branch(longitude, 38.020, 38.030)
                        for longitude in branch_longitudes
                    ],
                    "areaList": [],
                },
            },
        ],
    }


def _type2_single_branch_package() -> dict[str, Any]:
    """N=1 branch set followed by ACP/control-handover lines."""

    longitude = 127.050
    return {
        "inputMissionPackageID": PACKAGE_ID + 1,
        "inputMissionPackageType": 2,
        "availableAircraftList": [{"aircraftID": aircraft_id} for aircraft_id in UAV_IDS],
        "inputMissionList": [
            {
                "inputMissionID": SINGLE_BRANCH_MISSION_IDS[0],
                "inputMissionType": 1,
                "regionType": 7,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line_branch(longitude, 38.000, 38.010)],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": SINGLE_BRANCH_MISSION_IDS[1],
                "inputMissionType": 3,
                "regionType": 7,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [_area_branch(longitude)],
                },
            },
            {
                "inputMissionID": SINGLE_BRANCH_MISSION_IDS[2],
                "inputMissionType": 1,
                "regionType": 6,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line_branch(longitude, 38.020, 38.030)],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": 304,
                "inputMissionType": 1,
                "regionType": 3,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line_branch(longitude, 38.030, 38.040)],
                    "areaList": [],
                },
            },
            {
                "inputMissionID": 305,
                "inputMissionType": 1,
                "regionType": 2,
                "isDone": False,
                "missionDetail": {
                    "coordinateList": [],
                    "lineList": [_line_branch(longitude, 38.040, 38.050)],
                    "areaList": [],
                },
            },
        ],
    }


def _type2_eight_stage_package() -> dict[str, Any]:
    """Realistic Type-2 package with the self-reliance span at stages 4-6."""

    package = deepcopy(_type2_branch_package())
    branch_missions = package["inputMissionList"]

    def _single_line_mission(
        input_mission_id: int,
        *,
        region_type: int,
        start_latitude: float,
    ) -> dict[str, Any]:
        return {
            "inputMissionID": int(input_mission_id),
            "inputMissionType": 1,
            "regionType": int(region_type),
            "isDone": False,
            "missionDetail": {
                "coordinateList": [],
                "lineList": [_line_branch(126.900, start_latitude, start_latitude + 0.01)],
                "areaList": [],
            },
        }

    target_area = {
        "inputMissionID": 103,
        "inputMissionType": 3,
        "regionType": 6,
        "isDone": False,
        "missionDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": [_area_branch(126.950)],
        },
    }
    package["inputMissionList"] = [
        _single_line_mission(101, region_type=2, start_latitude=37.950),
        _single_line_mission(102, region_type=6, start_latitude=37.960),
        target_area,
        *branch_missions,
        _single_line_mission(107, region_type=3, start_latitude=38.040),
        _single_line_mission(108, region_type=2, start_latitude=38.050),
    ]
    return package


def _mission_reference(*, swapped: bool = False) -> dict[str, Any]:
    if swapped:
        # Move the previous branch-0 primary UAV4 onto branch 1 and UAV6 onto
        # branch 0. A position-based replan would therefore exchange owners.
        positions = {
            4: _coord(38.000, 127.100),
            5: _coord(38.000, 127.002),
            6: _coord(38.000, 127.000),
        }
    else:
        positions = {
            4: _coord(38.000, 127.000),
            5: _coord(38.000, 127.002),
            6: _coord(38.000, 127.100),
        }
    return {
        "missionReferencePackageID": PACKAGE_ID,
        "takeOverInfoList": [
            {"aircraftID": aircraft_id, "coordinate": deepcopy(positions[aircraft_id])}
            for aircraft_id in UAV_IDS
        ],
    }


@pytest.fixture
def isolated_branch_state(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> Path:
    state_path = tmp_path / "branch_ownership_state.json"
    monkeypatch.setattr(branch_ownership, "_state_path", lambda: state_path)
    return state_path


def _run_branch_plan(*, swapped: bool, apply_assignment: bool) -> SplitRunResult:
    return run_split_pipeline(
        deepcopy(_type2_branch_package()),
        deepcopy(_mission_reference(swapped=swapped)),
        list(UAV_IDS),
        apply_assignment=bool(apply_assignment),
        apply_scheduling=False,
    )


def _assigned_owners_by_branch(
    result: SplitRunResult,
    parent_order: int,
) -> dict[int, set[int]]:
    assigned: dict[int, set[int]] = {}
    for piece in result.pieces:
        if int(piece.parent_order) != int(parent_order):
            continue
        branch_index = int((piece.data or {}).get("branchIndex"))
        assert piece.assigned_uav is not None
        assigned.setdefault(branch_index, set()).add(int(piece.assigned_uav))
    return assigned


def test_three_uavs_two_branches_keep_one_fixed_group_for_line_area_and_return(
    isolated_branch_state: Path,
) -> None:
    result = _run_branch_plan(swapped=False, apply_assignment=True)

    ownership = {
        int(branch_index): [int(aircraft_id) for aircraft_id in owners]
        for branch_index, owners in result.branch_ownership.items()
    }
    assert result.branch_orders == [1, 2, 3]
    assert set(ownership) == {0, 1}
    assert sorted(len(owners) for owners in ownership.values()) == [1, 2]

    flattened_owners = [aircraft_id for owners in ownership.values() for aircraft_id in owners]
    assert sorted(flattened_owners) == UAV_IDS
    assert len(flattened_owners) == len(set(flattened_owners))

    expected_sets = {
        branch_index: set(owners) for branch_index, owners in ownership.items()
    }
    for parent_order in result.branch_orders:
        assert _assigned_owners_by_branch(result, parent_order) == expected_sets


def test_type2_guard_area_gives_every_sticky_owner_two_sequential_halves(
    isolated_branch_state: Path,
) -> None:
    result = _run_branch_plan(swapped=False, apply_assignment=True)

    area_pieces = [piece for piece in result.pieces if int(piece.parent_order) == 2]
    pieces_by_owner: dict[int, list] = {aircraft_id: [] for aircraft_id in UAV_IDS}
    for piece in area_pieces:
        assert piece.assigned_uav is not None
        pieces_by_owner[int(piece.assigned_uav)].append(piece)

    assert len(area_pieces) == 2 * len(UAV_IDS)
    for branch_index, owners in result.branch_ownership.items():
        for aircraft_id in owners:
            rows = sorted(
                pieces_by_owner[int(aircraft_id)],
                key=lambda piece: int((piece.data or {}).get("splitStage") or 0),
            )
            assert len(rows) == 2
            assert {
                int((piece.data or {}).get("branchIndex")) for piece in rows
            } == {int(branch_index)}
            assert [int((piece.data or {}).get("splitStage") or 0) for piece in rows] == [1, 2]
            assert [
                int((piece.data or {}).get("branchAreaSequence") or 0)
                for piece in rows
            ] == [1, 2]
            assert all(
                bool((piece.data or {}).get("branchAreaSequentialSplit"))
                for piece in rows
            )

    # The outbound and return LINE legs keep their original one-piece-per-owner
    # assignment; only the guard AREA receives the extra sequential half.
    for parent_order in (1, 3):
        line_pieces = [
            piece for piece in result.pieces if int(piece.parent_order) == parent_order
        ]
        assert len(line_pieces) == len(UAV_IDS)
        assert not any(
            bool((piece.data or {}).get("branchAreaSequentialSplit"))
            for piece in line_pieces
        )


def test_type3_guard_area_uses_the_same_two_half_self_reliance_contract(
    isolated_branch_state: Path,
) -> None:
    result = run_split_pipeline(
        deepcopy(_type2_branch_package(package_type=3)),
        deepcopy(_mission_reference(swapped=False)),
        list(UAV_IDS),
        apply_assignment=True,
        apply_scheduling=False,
    )

    area_pieces = [piece for piece in result.pieces if int(piece.parent_order) == 2]
    assert len(area_pieces) == 2 * len(UAV_IDS)
    for aircraft_id in UAV_IDS:
        rows = sorted(
            [piece for piece in area_pieces if int(piece.assigned_uav or 0) == aircraft_id],
            key=lambda piece: int((piece.data or {}).get("splitStage") or 0),
        )
        assert [int((piece.data or {}).get("splitStage") or 0) for piece in rows] == [1, 2]


def test_guard_area_degenerate_axis_still_falls_back_to_two_unique_halves(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo import (
        split_algorithms,
    )

    mission = deepcopy(_type2_branch_package()["inputMissionList"][1])
    mission["missionDetail"]["areaList"] = mission["missionDetail"]["areaList"][:1]

    def _force_single_stage_fallback(
        area_poly: list[dict[str, Any]],
        uav_cnt: int,
        **kwargs: Any,
    ) -> list[dict]:
        return split_algorithms.divide_search_area_clip(
            area_poly,
            uav_cnt,
            float(kwargs["entry_move_bearing_deg"]),
        )

    monkeypatch.setattr(
        split_algorithms,
        "divide_search_area_two_stage",
        _force_single_stage_fallback,
    )

    rows = split_algorithms.split_mission_into_subareas(
        mission,
        1,
        _coord(38.000, 127.000),
        _coord(38.030, 127.000),
        [1],
        split_branch_area_into_two=True,
    )

    assert len(rows) == 2
    assert [int(row.get("splitStage") or 0) for row in rows] == [1, 2]
    assert [int(row.get("branchAreaSequence") or 0) for row in rows] == [1, 2]
    assert all(bool(row.get("branchAreaSequentialSplit")) for row in rows)


def test_type2_guard_area_replan_keeps_each_owner_and_emits_two_rows(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )

    target_mission = deepcopy(_type2_branch_package()["inputMissionList"][1])
    target_mission["missionDetail"]["areaList"].append(_area_branch(127.200))
    ownership = {0: [6], 1: [5], 2: [4]}
    planner_owner_pairs: list[tuple[int, int]] = []
    reserved_rows: list[dict[int, list[dict[str, Any]]]] = []

    def _fake_planner(**kwargs: Any) -> SimpleNamespace:
        entries = list(kwargs["aircraft_entries"])
        assert len(entries) == 2
        real_id = int(entries[0]["aircraftID"])
        virtual_id = int(entries[1]["aircraftID"])
        assert real_id in UAV_IDS
        assert virtual_id not in UAV_IDS
        planner_owner_pairs.append((real_id, virtual_id))
        real_xy = next_collab_pipeline.coord_to_xy(entries[0]["coordinate"])
        assert real_xy is not None
        x, y = float(real_xy[0]), float(real_xy[1])
        return SimpleNamespace(
            expected_paths=[
                {
                    "aircraftID": real_id,
                    "pieceIndex": 1,
                    "waypointStartXY": (x, y),
                    "waypointEndXY": (x + 100.0, y),
                    "routeXY": [(x, y), (x + 100.0, y)],
                },
                {
                    "aircraftID": virtual_id,
                    "pieceIndex": 2,
                    "waypointStartXY": (x + 100.0, y),
                    "waypointEndXY": (x + 200.0, y),
                    "routeXY": [(x + 100.0, y), (x + 200.0, y)],
                },
            ],
            split_result=SimpleNamespace(pieces=[]),
            mid_line_segments=[],
            workflow="type2-boundary-two-piece-test",
            planner_result_text="",
        )

    def _stop_at_reservation(**kwargs: Any) -> None:
        reserved_rows.append(deepcopy(kwargs["path_rows_by_aircraft"]))
        return None

    monkeypatch.setattr(
        next_collab_pipeline,
        "_branch_area_ownership_for_target",
        lambda *_args, **_kwargs: deepcopy(ownership),
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "run_next_collab_division_plan",
        _fake_planner,
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "_prewarm_dem_altitudes_for_path_rows_if_enabled",
        lambda *_args, **_kwargs: {"uniquePairs": 0},
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "_reserve_next_collab_replacement_ids",
        _stop_at_reservation,
    )

    messages: list[str] = []
    result = next_collab_pipeline._prepare_area_replacements(
        target_input_mission=target_mission,
        target_input_id=int(target_mission["inputMissionID"]),
        target_aircraft_ids=list(UAV_IDS),
        entry_coord_map={
            4: _coord(38.000, 127.198),
            5: _coord(38.000, 127.098),
            6: _coord(38.000, 126.998),
        },
        heading_map={4: 0.0, 5: 0.0, 6: 0.0},
        entry_aircraft_context_map=None,
        representative_entry=None,
        template_record_map={},
        now_ms=1,
        turn_radius_scale=1.0,
        emit=messages.append,
        planning_mode={"package_type": 2},
    )

    assert result is None  # Deliberately stopped at the ID reservation boundary.
    # Every component is planned twice: once to learn where the first piece
    # ends, then again with the planning-only partner re-seeded at that exit.
    assert [real_id for real_id, _virtual_id in planner_owner_pairs] == [6, 6, 5, 5, 4, 4]
    assert len(reserved_rows) == 1
    assert set(reserved_rows[0]) == set(UAV_IDS)
    for aircraft_id in UAV_IDS:
        rows = reserved_rows[0][aircraft_id]
        assert len(rows) == 2
        assert [int(row["aircraftID"]) for row in rows] == [aircraft_id, aircraft_id]
        assert [int(row["areaSingleAircraftSequence"]) for row in rows] == [1, 2]
        assert all(bool(row["areaSingleAircraftSequentialSplit"]) for row in rows)
        assert isinstance(rows[1].get("areaPassEntryCoordinate"), dict)
        assert rows[1]["areaPassEntryCoordinate"] != rows[0]["areaPassEntryCoordinate"]
    assert all(
        int(row["aircraftID"]) in UAV_IDS
        for rows in reserved_rows[0].values()
        for row in rows
    )
    assert sum("boundary component" in message for message in messages) == 3


def test_type2_shared_boundary_replan_keeps_two_halves_per_co_owner(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )

    target_mission = deepcopy(_type2_branch_package()["inputMissionList"][1])
    ownership = {0: [4, 5], 1: [6]}
    planner_entry_counts: list[int] = []
    reserved_rows: list[dict[int, list[dict[str, Any]]]] = []

    def _fake_planner(**kwargs: Any) -> SimpleNamespace:
        entries = list(kwargs["aircraft_entries"])
        planner_entry_counts.append(len(entries))
        expected_paths = []
        for piece_index, entry in enumerate(entries, start=1):
            start_xy = next_collab_pipeline.coord_to_xy(entry["coordinate"])
            assert start_xy is not None
            x, y = float(start_xy[0]), float(start_xy[1])
            expected_paths.append(
                {
                    "aircraftID": int(entry["aircraftID"]),
                    "pieceIndex": int(piece_index),
                    "waypointStartXY": (x, y),
                    "waypointEndXY": (x + 25.0, y + float(piece_index)),
                    "routeXY": [(x, y), (x + 25.0, y + float(piece_index))],
                }
            )
        return SimpleNamespace(
            expected_paths=expected_paths,
            split_result=SimpleNamespace(pieces=[]),
            mid_line_segments=[],
            workflow="type2-shared-boundary-two-piece-test",
            planner_result_text="",
        )

    def _stop_at_reservation(**kwargs: Any) -> None:
        reserved_rows.append(deepcopy(kwargs["path_rows_by_aircraft"]))
        return None

    monkeypatch.setattr(
        next_collab_pipeline,
        "_branch_area_ownership_for_target",
        lambda *_args, **_kwargs: deepcopy(ownership),
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "run_next_collab_division_plan",
        _fake_planner,
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "_prewarm_dem_altitudes_for_path_rows_if_enabled",
        lambda *_args, **_kwargs: {"uniquePairs": 0},
    )
    monkeypatch.setattr(
        next_collab_pipeline,
        "_reserve_next_collab_replacement_ids",
        _stop_at_reservation,
    )

    result = next_collab_pipeline._prepare_area_replacements(
        target_input_mission=target_mission,
        target_input_id=int(target_mission["inputMissionID"]),
        target_aircraft_ids=list(UAV_IDS),
        entry_coord_map={
            4: _coord(38.000, 126.998),
            5: _coord(38.000, 127.002),
            6: _coord(38.000, 127.098),
        },
        heading_map={4: 0.0, 5: 0.0, 6: 0.0},
        entry_aircraft_context_map=None,
        representative_entry=None,
        template_record_map={},
        now_ms=1,
        turn_radius_scale=1.0,
        emit=lambda _message: None,
        planning_mode={"package_type": 2},
    )

    assert result is None
    # Each component runs a second pass that re-seeds the planning-only
    # partners at their owners' first-piece exits.
    assert planner_entry_counts == [4, 4, 2, 2]
    assert len(reserved_rows) == 1
    for aircraft_id in UAV_IDS:
        rows = reserved_rows[0][aircraft_id]
        assert len(rows) == 2
        assert [int(row["areaSingleAircraftSequence"]) for row in rows] == [1, 2]
        assert all(int(row["aircraftID"]) == aircraft_id for row in rows)
    assert {
        aircraft_id: {int(row["areaComponentIndex"]) for row in rows}
        for aircraft_id, rows in reserved_rows[0].items()
    } == {4: {1}, 5: {1}, 6: {2}}


def test_read_only_replan_with_swapped_positions_keeps_initial_ownership(
    isolated_branch_state: Path,
) -> None:
    initial = _run_branch_plan(swapped=False, apply_assignment=True)
    replanned = _run_branch_plan(swapped=True, apply_assignment=False)

    assert replanned.branch_orders == initial.branch_orders
    assert replanned.branch_ownership == initial.branch_ownership
    assert branch_ownership.get_branch_ownership(PACKAGE_ID) == initial.branch_ownership


def test_repeated_assignment_for_same_package_keeps_initial_ownership(
    isolated_branch_state: Path,
) -> None:
    initial = _run_branch_plan(swapped=False, apply_assignment=True)
    repeated = _run_branch_plan(swapped=True, apply_assignment=True)

    assert repeated.branch_orders == initial.branch_orders
    assert repeated.branch_ownership == initial.branch_ownership
    assert branch_ownership.get_branch_ownership(PACKAGE_ID) == initial.branch_ownership


def test_type2_replan_without_authoritative_state_fails_closed(
    isolated_branch_state: Path,
) -> None:
    with pytest.raises(RuntimeError, match="immutable branch ownership is unavailable"):
        _run_branch_plan(swapped=True, apply_assignment=False)


def _replan_without(*absent_aircraft: int):
    """Replan the established package with some UAVs permanently gone."""

    remaining = [aid for aid in (4, 5, 6) if aid not in set(absent_aircraft)]
    result = run_split_pipeline(
        deepcopy(_type2_branch_package()),
        deepcopy(_mission_reference(swapped=True)),
        remaining,
        apply_assignment=True,
        apply_scheduling=False,
    )
    owners_by_branch: dict[int, set[int]] = {}
    for piece in result.pieces:
        data = piece.data if isinstance(piece.data, dict) else {}
        branch_index = data.get("branchIndex")
        if branch_index is None:
            continue
        owners_by_branch.setdefault(int(branch_index), set()).add(piece.assigned_uav)
    return result, owners_by_branch


def test_a_sole_owner_leaving_takes_its_whole_branch_set_out(
    isolated_branch_state: Path,
) -> None:
    """A UAV gone for good removes its branch set, and nothing else.

    The whole package used to be deferred, which cost the replan of every
    unrelated mission too. Now only the orphaned element leaves the plan.
    """

    _run_branch_plan(swapped=False, apply_assignment=True)
    result, owners_by_branch = _replan_without(6)  # sole owner of branch 1

    assert result.branch_ownership == {0: [4, 5], 1: [6]}
    assert result.dropped_branches == [1]
    # The dropped branch produced no geometry at all...
    assert 1 not in owners_by_branch
    # ...and nobody inherited it.
    assert owners_by_branch[0] == {4, 5}


def test_a_branch_keeping_one_co_owner_is_still_flown(
    isolated_branch_state: Path,
) -> None:
    """Losing a co-owner must not delete a branch someone is still flying."""

    _run_branch_plan(swapped=False, apply_assignment=True)
    result, owners_by_branch = _replan_without(5)  # co-owner of branch 0

    assert result.dropped_branches == []
    # Branch 0 keeps flying, now as a single slice for its surviving owner.
    assert owners_by_branch[0] == {4}
    assert owners_by_branch[1] == {6}


def _branch_zero_slices(result) -> list[tuple[int, float]]:
    """(aircraftID, width) of every slice cut from branch 0's corridor."""

    out = []
    for piece in result.pieces:
        data = piece.data if isinstance(piece.data, dict) else {}
        if data.get("branchIndex") != 0 or data.get("MissionID") != 201:
            continue
        width = data.get("width", data.get("Width"))
        out.append((piece.assigned_uav, float(width)))
    return sorted(out)


def test_the_surviving_co_owner_takes_over_the_whole_branch(
    isolated_branch_state: Path,
) -> None:
    """One aircraft then covers the branch alone, at full corridor width.

    The takeover is strictly inside the branch - its own co-owner absorbs the
    share. No UAV from another branch ever touches it, which is the part of the
    각자도생 contract that must not bend.
    """

    _run_branch_plan(swapped=False, apply_assignment=True)

    shared = _branch_zero_slices(_replan_without()[0])
    alone = _branch_zero_slices(_replan_without(5)[0])

    # Two owners: the corridor is halved, one slice each.
    assert [aircraft for aircraft, _width in shared] == [4, 5]
    assert len({width for _aircraft, width in shared}) == 1
    half = shared[0][1]

    # One owner left: a single slice covering the full width, not just its half.
    assert [aircraft for aircraft, _width in alone] == [4]
    assert alone[0][1] == pytest.approx(half * 2.0)


def test_a_dropout_no_longer_stalls_the_whole_package(
    isolated_branch_state: Path,
) -> None:
    """The point of the change: one absent UAV must not defer everything.

    This used to raise, which lost the replan of every mission in the package -
    branch or not - for that cycle.
    """

    _run_branch_plan(swapped=False, apply_assignment=True)
    full, _ = _replan_without()
    reduced, _ = _replan_without(6)

    def missions(result) -> set:
        return {(piece.data or {}).get("MissionID") for piece in result.pieces}

    # Every mission is still planned; only branch 1's share of them is gone.
    assert missions(reduced) == missions(full)
    assert len(reduced.pieces) < len(full.pieces)


def test_a_branch_that_keeps_one_owner_survives(isolated_branch_state: Path) -> None:
    """Losing a co-owner must not delete a branch someone is still flying."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _surviving_branch_ownership,
    )

    surviving, dropped = _surviving_branch_ownership({0: [4, 6], 1: [5]}, [4, 5])

    assert surviving == {0: [4], 1: [5]}
    assert dropped == []


def test_every_owner_gone_drops_the_branch(isolated_branch_state: Path) -> None:
    from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
        _surviving_branch_ownership,
    )

    surviving, dropped = _surviving_branch_ownership({0: [4], 1: [5], 2: [6]}, [4])

    assert surviving == {0: [4], 1: [], 2: []}
    assert dropped == [1, 2]


def test_split_result_cache_payload_roundtrip_preserves_branch_contract() -> None:
    from modules.mission_planning.runtime import next_collab_line_runner

    original = SplitRunResult(
        uav_count=3,
        uav_ids=list(UAV_IDS),
        branch_ownership={0: [4, 5], 1: [6]},
        branch_orders=[1, 2, 3],
    )

    payload = next_collab_line_runner._split_result_to_payload(original)
    restored = next_collab_line_runner._split_result_from_payload(payload)

    assert restored.branch_ownership == original.branch_ownership
    assert restored.branch_orders == original.branch_orders


def test_attack_tracking_preserves_only_locked_type2_branch_ownership(
    isolated_branch_state: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline

    # Seed the authoritative package/branch state exactly as the initial plan does.
    _run_branch_plan(swapped=False, apply_assignment=True)
    selected_plan = {"payload": _type2_branch_package(package_type=2)}
    monkeypatch.setattr(
        attack_pipeline,
        "_load_input_plan_for_source_plan",
        lambda _source_plan_id: deepcopy(selected_plan["payload"]),
    )

    assert (
        attack_pipeline._attack_tracking_collab_remaining_policy(
            source_plan_id=700_000_001,
            input_mission_id=BRANCH_MISSION_IDS[1],
        )
        == "preserve"
    )

    selected_plan["payload"] = _type2_branch_package(package_type=1)
    assert (
        attack_pipeline._attack_tracking_collab_remaining_policy(
            source_plan_id=700_000_001,
            input_mission_id=BRANCH_MISSION_IDS[1],
        )
        == "redivide"
    )


def test_type2_rebuilds_only_branch_line_suffixes() -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        TYPE2_SELF_RELIANCE_GUARD_AREA,
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    )
    from modules.mission_planning.replanning.triggers.post_attack import (
        pipeline as post_attack_pipeline,
    )

    assert attack_pipeline._attack_resume_descriptor_uav_ids(
        configured_reuse=True,
        other_uav_ids=[4, 5, 6],
        current_input_by_aircraft={4: 201, 5: 202, 6: 103},
        collaborative_input_ids=[103],
        type2_branch_line_aircraft_ids=[4],
    ) == {4, 6}
    assert attack_pipeline._attack_resume_descriptor_uav_ids(
        configured_reuse=True,
        other_uav_ids=[4, 5, 6],
        current_input_by_aircraft={4: 201, 5: 202, 6: 103},
        collaborative_input_ids=[],
        type2_branch_line_aircraft_ids=[4],
    ) == {4}
    assert attack_pipeline._attack_resume_descriptor_uav_ids(
        configured_reuse=False,
        other_uav_ids=[4, 5, 6],
        current_input_by_aircraft={4: 201, 5: 202, 6: 103},
        collaborative_input_ids=[],
        type2_branch_line_aircraft_ids=[4],
    ) == {4, 5, 6}
    assert post_attack_pipeline._requires_type2_individual_suffix_refresh(
        "type2_branch_owner_resume_preserved",
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    ) is True
    assert post_attack_pipeline._requires_type2_individual_suffix_refresh(
        "type2_branch_owner_resume_preserved",
        TYPE2_SELF_RELIANCE_GUARD_AREA,
    ) is False
    assert post_attack_pipeline._requires_type2_individual_suffix_refresh(
        "active_group_progress_high",
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    ) is False


def test_attack_branch_line_advances_only_on_authoritative_completion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline

    monkeypatch.setattr(
        attack_pipeline,
        "_source_type2_self_reliance_phase",
        lambda **_kwargs: "outbound_line",
    )
    complete = {
        500000001: {
            "sweep_point_count": 6,
            "progress_points": 6,
            "progress_percent": 100,
            "remaining_seconds": 0,
        }
    }
    partial = deepcopy(complete)
    partial[500000001]["progress_points"] = 2
    partial[500000001]["progress_percent"] = 33
    partial[500000001]["remaining_seconds"] = 40

    assert attack_pipeline._type2_branch_line_completion_confirmed(
        source_plan_id=700000001,
        input_mission_id=BRANCH_MISSION_IDS[0],
        path_id=500000001,
        sweep_progress=complete,
    ) is True
    assert attack_pipeline._type2_branch_line_completion_confirmed(
        source_plan_id=700000001,
        input_mission_id=BRANCH_MISSION_IDS[0],
        path_id=500000001,
        sweep_progress=partial,
    ) is False
    assert attack_pipeline._type2_branch_line_completion_confirmed(
        source_plan_id=700000001,
        input_mission_id=BRANCH_MISSION_IDS[0],
        path_id=500000001,
        sweep_progress={},
    ) is False


def test_single_branch_profile_stops_before_acp_and_control_handover_lines() -> None:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        detect_ground_maneuver_profile,
    )

    profile = detect_ground_maneuver_profile(_type2_single_branch_package())

    assert profile is not None
    assert profile["branchCount"] == 1
    assert profile["branchOrders"] == [1, 2, 3]
    assert profile["branchInputMissionIDs"] == list(SINGLE_BRANCH_MISSION_IDS)
    assert 304 not in profile["branchInputMissionIDs"]
    assert 305 not in profile["branchInputMissionIDs"]


def test_type2_self_reliance_phase_resolver_only_matches_exact_branch_span() -> None:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        TYPE2_SELF_RELIANCE_GUARD_AREA,
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
        TYPE2_SELF_RELIANCE_RETURN_LINE,
        resolve_type2_self_reliance_phase,
    )

    package = _type2_eight_stage_package()

    assert resolve_type2_self_reliance_phase(package, BRANCH_MISSION_IDS[0]) == (
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE
    )
    assert resolve_type2_self_reliance_phase(package, BRANCH_MISSION_IDS[1]) == (
        TYPE2_SELF_RELIANCE_GUARD_AREA
    )
    assert resolve_type2_self_reliance_phase(package, BRANCH_MISSION_IDS[2]) == (
        TYPE2_SELF_RELIANCE_RETURN_LINE
    )
    for normal_input_mission_id in (101, 102, 103, 107, 108, 999_999):
        assert resolve_type2_self_reliance_phase(package, normal_input_mission_id) is None


def test_type2_self_reliance_phase_resolver_rejects_stale_or_malformed_profile() -> None:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        resolve_type2_self_reliance_phase,
    )

    wrong_kind = _type2_eight_stage_package()
    outbound = wrong_kind["inputMissionList"][3]
    outbound["inputMissionType"] = 3
    outbound["missionDetail"]["lineList"] = []
    outbound["missionDetail"]["areaList"] = [
        _area_branch(longitude) for longitude in (127.000, 127.100)
    ]
    assert resolve_type2_self_reliance_phase(wrong_kind, BRANCH_MISSION_IDS[0]) is None

    unequal_branch_count = _type2_eight_stage_package()
    unequal_branch_count["inputMissionList"][4]["missionDetail"]["areaList"] = (
        unequal_branch_count["inputMissionList"][4]["missionDetail"]["areaList"][:1]
    )
    assert (
        resolve_type2_self_reliance_phase(
            unequal_branch_count,
            BRANCH_MISSION_IDS[0],
        )
        is None
    )

    legacy_without_regions = _type2_eight_stage_package()
    for mission in legacy_without_regions["inputMissionList"]:
        mission.pop("regionType", None)
    assert (
        resolve_type2_self_reliance_phase(
            legacy_without_regions,
            BRANCH_MISSION_IDS[0],
        )
        is None
    )

    wrong_package_type = _type2_eight_stage_package()
    wrong_package_type["inputMissionPackageType"] = 3
    assert (
        resolve_type2_self_reliance_phase(
            wrong_package_type,
            BRANCH_MISSION_IDS[0],
        )
        is None
    )


def test_attack_and_post_attack_scope_helpers_exclude_normal_target_missions(
    isolated_branch_state: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        TYPE2_SELF_RELIANCE_GUARD_AREA,
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
        TYPE2_SELF_RELIANCE_RETURN_LINE,
    )
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline
    from modules.mission_planning.replanning.triggers.post_attack import (
        pipeline as post_attack_pipeline,
    )

    _run_branch_plan(swapped=False, apply_assignment=True)
    package = _type2_eight_stage_package()
    monkeypatch.setattr(
        attack_pipeline,
        "_load_input_plan_for_source_plan",
        lambda _source_plan_id: deepcopy(package),
    )
    monkeypatch.setattr(
        post_attack_pipeline,
        "_load_input_plan_for_source_plan",
        lambda _source_plan_id: deepcopy(package),
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_source_input_mission_is_locked_type2_branch",
        lambda **_kwargs: True,
    )
    monkeypatch.setattr(
        post_attack_pipeline,
        "_source_input_mission_is_locked_type2_branch",
        lambda **_kwargs: True,
    )

    for helper in (
        attack_pipeline._source_type2_self_reliance_phase,
        post_attack_pipeline._source_type2_self_reliance_phase,
    ):
        assert helper(source_plan_id=700_000_002, input_mission_id=102) is None
        assert helper(source_plan_id=700_000_002, input_mission_id=103) is None
        assert helper(
            source_plan_id=700_000_002,
            input_mission_id=BRANCH_MISSION_IDS[0],
        ) == TYPE2_SELF_RELIANCE_OUTBOUND_LINE
        assert helper(
            source_plan_id=700_000_002,
            input_mission_id=BRANCH_MISSION_IDS[1],
        ) == TYPE2_SELF_RELIANCE_GUARD_AREA
        assert helper(
            source_plan_id=700_000_002,
            input_mission_id=BRANCH_MISSION_IDS[2],
        ) == TYPE2_SELF_RELIANCE_RETURN_LINE

    monkeypatch.setattr(
        attack_pipeline,
        "_source_input_mission_is_locked_type2_branch",
        lambda *, source_plan_id, input_mission_id: int(input_mission_id)
        in set(BRANCH_MISSION_IDS),
    )
    assert attack_pipeline._attack_tracking_collab_remaining_policy(
        source_plan_id=700_000_002,
        input_mission_id=103,
    ) == "redivide"
    assert attack_pipeline._attack_tracking_collab_remaining_policy(
        source_plan_id=700_000_002,
        input_mission_id=BRANCH_MISSION_IDS[1],
    ) == "preserve"


def test_single_branch_assigns_all_uavs_only_to_that_branch(
    isolated_branch_state: Path,
) -> None:
    result = run_split_pipeline(
        deepcopy(_type2_single_branch_package()),
        deepcopy(_mission_reference(swapped=False)),
        list(UAV_IDS),
        apply_assignment=True,
        apply_scheduling=False,
    )

    assert result.branch_orders == [1, 2, 3]
    assert set(result.branch_ownership) == {0}
    assert sorted(result.branch_ownership[0]) == UAV_IDS
    assert {
        int((piece.data or {}).get("branchIndex")) for piece in result.pieces
        if int(piece.parent_order) in result.branch_orders
    } == {0}


def test_second_registration_cannot_replace_first_branch_ownership(
    isolated_branch_state: Path,
) -> None:
    initial = {0: [4, 5], 1: [6]}
    replacement = {0: [6], 1: [4, 5]}

    first_result = branch_ownership.register_branch_ownership(
        package_id=PACKAGE_ID,
        branch_count=2,
        ownership=initial,
        branch_mission_ids=BRANCH_MISSION_IDS,
        anchor_input_mission_id=BRANCH_MISSION_IDS[0],
        source="initial-plan",
    )
    second_result = branch_ownership.register_branch_ownership(
        package_id=PACKAGE_ID,
        branch_count=2,
        ownership=replacement,
        branch_mission_ids=reversed(BRANCH_MISSION_IDS),
        anchor_input_mission_id=BRANCH_MISSION_IDS[-1],
        source="replan",
    )

    assert first_result == initial
    assert second_result == initial
    assert branch_ownership.get_branch_ownership(PACKAGE_ID) == initial
    meta = branch_ownership.get_branch_meta(PACKAGE_ID)
    assert meta["branch_mission_ids"] == list(BRANCH_MISSION_IDS)
    assert meta["ownership_locked"] is True


def test_update_api_cannot_replace_first_branch_ownership(
    isolated_branch_state: Path,
) -> None:
    initial = {0: [4, 5], 1: [6]}
    replacement = {0: [6], 1: [4, 5]}
    branch_ownership.register_branch_ownership(
        package_id=PACKAGE_ID,
        branch_count=2,
        ownership=initial,
        branch_mission_ids=BRANCH_MISSION_IDS,
        anchor_input_mission_id=BRANCH_MISSION_IDS[0],
        source="initial-plan",
    )

    update_result = branch_ownership.update_branch_ownership(
        PACKAGE_ID,
        replacement,
        source="legacy-replan",
    )

    assert update_result == initial
    assert branch_ownership.get_branch_ownership(PACKAGE_ID) == initial


def test_prior_and_post_attack_guards_recognize_only_locked_type2_branch_missions(
    isolated_branch_state: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.post_attack import (
        pipeline as post_attack_pipeline,
    )
    from modules.mission_planning.replanning.triggers.prior import pipeline as prior_pipeline

    _run_branch_plan(swapped=False, apply_assignment=True)
    selected_plan = {"payload": _type2_branch_package(package_type=2)}
    monkeypatch.setattr(
        prior_pipeline,
        "_load_input_plan_for_source_plan",
        lambda _source_plan_id: deepcopy(selected_plan["payload"]),
    )

    for guard in (
        prior_pipeline._source_input_mission_is_locked_type2_branch,
        post_attack_pipeline._source_input_mission_is_locked_type2_branch,
    ):
        assert guard(700_000_001, BRANCH_MISSION_IDS[1]) is True
        assert guard(700_000_001, 999_999) is False

    selected_plan["payload"] = _type2_branch_package(package_type=1)
    assert (
        prior_pipeline._source_input_mission_is_locked_type2_branch(
            700_000_001,
            BRANCH_MISSION_IDS[1],
        )
        is False
    )


def test_post_attack_type2_return_skips_team_redistribution(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from modules.mission_planning.replanning.triggers.post_attack import (
        pipeline as post_attack_pipeline,
    )

    monkeypatch.setattr(
        post_attack_pipeline,
        "_aircraft_ids_for_input_mission",
        lambda **_kwargs: [4, 5, 6],
    )
    monkeypatch.setattr(
        post_attack_pipeline,
        "list_active_tracking_assignments",
        lambda: [],
    )
    monkeypatch.setattr(
        post_attack_pipeline,
        "_source_input_mission_is_locked_type2_branch",
        lambda *_args, **_kwargs: True,
    )

    evaluation = post_attack_pipeline._evaluate_rejoin_group(
        current_plan_id=700_000_101,
        current_input_id=BRANCH_MISSION_IDS[1],
        group_assignments=[
            {"aircraft_id": 4, "attack_plan_id": 700_000_101}
        ],
        agent_state_map={},
        config={},
        emit=lambda _message: None,
    )

    assert evaluation["replan_needed"] is False
    assert evaluation["skip_reason"] == "type2_branch_owner_resume_preserved"
    assert evaluation["active_aircraft_ids"] == [5, 6]
    assert evaluation["returning_aircraft_ids"] == [4]


def _sequential_context(real_id: int, virtual_id: int, real_xy: tuple[float, float]):
    return {
        "realAircraftID": int(real_id),
        "virtualAircraftID": int(virtual_id),
        "realEntryCoordinate": _coord(38.0, 127.0),
        "realEntryXY": (float(real_xy[0]), float(real_xy[1])),
    }


def test_sequential_second_piece_entry_moves_to_first_piece_exit() -> None:
    """The partner that plans piece 2 must start where piece 1 ended.

    Leaving it beside the real UAV is what made the second Area film from a
    long standoff: its ingress was solved against the aircraft's replan-time
    position instead of the track it had just flown.
    """

    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )

    real_xy = (0.0, 0.0)
    first_end_xy = (1200.0, 400.0)
    planner_result = SimpleNamespace(
        expected_paths=[
            {
                "aircraftID": 4,
                "waypointStartXY": (100.0, 0.0),
                "waypointEndXY": first_end_xy,
                "routeXY": [(100.0, 0.0), (700.0, 200.0), first_end_xy],
            },
            {
                "aircraftID": 1_000_004,
                "waypointStartXY": (2000.0, 900.0),
                "waypointEndXY": (2600.0, 1200.0),
                "routeXY": [(2000.0, 900.0), (2600.0, 1200.0)],
            },
        ]
    )
    context = _sequential_context(4, 1_000_004, real_xy)
    planner_entries = [
        {"aircraftID": 4, "coordinate": _coord(38.0, 127.0), "headingDeg": 10.0},
        {
            "aircraftID": 1_000_004,
            "coordinate": _coord(38.0, 127.0),
            "headingDeg": 10.0,
            "turnSign": 1,
        },
    ]

    updated = next_collab_pipeline._sequential_area_second_pass_entries(
        planner_result,
        planner_entries,
        [context],
    )
    assert updated is not None
    assert updated[0]["coordinate"] == planner_entries[0]["coordinate"]  # real UAV untouched

    moved_xy = next_collab_pipeline.coord_to_xy(updated[1]["coordinate"])
    assert moved_xy is not None
    assert abs(float(moved_xy[0]) - first_end_xy[0]) < 1.0
    assert abs(float(moved_xy[1]) - first_end_xy[1]) < 1.0
    assert context["secondEntryXY"] == first_end_xy
    # A stale live-turn prediction would drag the partner back to the aircraft.
    assert "turnSign" not in updated[1]

    # Heading follows the first piece's exit leg (700,200) -> (1200,400).
    assert abs(float(updated[1]["headingDeg"]) - 68.198) < 0.5

    assert next_collab_pipeline._sequential_area_second_pass_is_consistent(
        planner_result,
        [context],
    )


def test_sequential_second_pass_rejected_when_piece_order_inverts() -> None:
    """A re-plan that hands the near piece to the partner must be discarded."""

    from modules.mission_planning.replanning.triggers.next_collab import (
        pipeline as next_collab_pipeline,
    )

    inverted = SimpleNamespace(
        expected_paths=[
            {
                "aircraftID": 4,
                "waypointStartXY": (2000.0, 900.0),
                "waypointEndXY": (2600.0, 1200.0),
                "routeXY": [(2000.0, 900.0), (2600.0, 1200.0)],
            },
            {
                "aircraftID": 1_000_004,
                "waypointStartXY": (100.0, 0.0),
                "waypointEndXY": (1200.0, 400.0),
                "routeXY": [(100.0, 0.0), (1200.0, 400.0)],
            },
        ]
    )
    context = _sequential_context(4, 1_000_004, (0.0, 0.0))
    assert not next_collab_pipeline._sequential_area_second_pass_is_consistent(
        inverted,
        [context],
    )
