"""Type 2 각자도생: each branch keeps its own sweep direction on replan.

A normal LINE mission is one corridor split across the UAVs, so a replan pins
every piece to a single "first execution" deployment direction and they all
sweep the same way. A 각자도생 mission is different: its lineList holds several
independent corridors, each with its own heading. Imposing one direction on all
of them reverses any branch more than 90 degrees off it, and the operator sees a
single UAV flying its corridor backwards while the others look right.
"""

from __future__ import annotations

import inspect
import math
from types import SimpleNamespace

from modules.mission_planning.replanning.triggers.next_collab import pipeline
from modules.mission_planning.MissionPlanner.planning_enhanced.models import SplitPiece
from modules.mission_planning.runtime.next_collab_line_runner import (
    _type2_branch_deployment_reference,
)


# The three 경계지역 corridors of the reported package, as declared in 0201.
DECLARED_BEARINGS = {0: 320.9, 1: 15.5, 2: 53.8}


def _reversed_by_shared_direction(shared_deg: float) -> set[int]:
    """Branches a single mission-wide deployment direction would flip."""

    flipped = set()
    for branch, bearing in DECLARED_BEARINGS.items():
        offset = abs((bearing - shared_deg + 180.0) % 360.0 - 180.0)
        if offset > 90.0:
            flipped.add(branch)
    return flipped


def test_a_shared_direction_would_reverse_a_branch() -> None:
    """Why the guard is needed: branch 2 sits 92.9 degrees off branch 0."""

    assert _reversed_by_shared_direction(DECLARED_BEARINGS[0]) == {2}
    offset = abs(
        (DECLARED_BEARINGS[2] - DECLARED_BEARINGS[0] + 180.0) % 360.0 - 180.0
    )
    assert math.isclose(offset, 92.9, abs_tol=0.1)


def test_branch_missions_skip_the_mission_wide_deployment_direction() -> None:
    source = inspect.getsource(pipeline._prepare_line_replacements)
    resolver = "_resolve_line_deployment_coordinate_list_from_templates"

    assert resolver in source
    # The resolver must be reachable only when the mission is not a locked
    # Type-2 branch mission.
    guarded = f"if locked_type2_ownership\n        else {resolver}("
    assert guarded in source, "branch missions must bypass the shared direction"


def test_non_branch_line_missions_still_pin_their_direction() -> None:
    """The carry-forward is what stops an ordinary corridor reversing."""

    source = inspect.getsource(pipeline._prepare_line_replacements)

    assert "lineDeploymentDirectionLocked" in source
    assert "first execution deployment direction restored from prior plan." in source


def test_reexecute_line_uses_original_branch_id_only_for_split(monkeypatch) -> None:
    source_input_id = 201
    target_input_id = 999
    ownership = {0: [4, 5], 1: [6]}
    planner_input_ids: list[int] = []
    messages: list[str] = []

    def _fake_line_plan(**kwargs):
        planner_input_ids.append(int(kwargs["target_mission"]["inputMissionID"]))
        return SimpleNamespace(
            expected_paths=[],
            split_result=SimpleNamespace(
                branch_ownership=ownership,
                pieces=[
                    SimpleNamespace(data={"branchIndex": 0}, assigned_uav=4),
                    SimpleNamespace(data={"branchIndex": 0}, assigned_uav=5),
                    SimpleNamespace(data={"branchIndex": 1}, assigned_uav=6),
                ],
            ),
        )

    monkeypatch.setattr(pipeline, "run_next_collab_line_plan", _fake_line_plan)
    target_mission = {
        "inputMissionID": target_input_id,
        "missionDetail": {
            "lineList": [
                {
                    "width": 900.0,
                    "coordinateList": [
                        {"latitude": 38.0, "longitude": 127.0, "altitude": 700.0},
                        {"latitude": 38.1, "longitude": 127.0, "altitude": 700.0},
                    ],
                },
                {
                    "width": 900.0,
                    "coordinateList": [
                        {"latitude": 38.0, "longitude": 127.1, "altitude": 700.0},
                        {"latitude": 38.1, "longitude": 127.1, "altitude": 700.0},
                    ],
                },
            ]
        },
    }

    result = pipeline._prepare_line_replacements(
        target_input_mission=target_mission,
        target_input_id=target_input_id,
        target_aircraft_ids=[4, 5, 6],
        entry_coord_map={
            4: {"latitude": 38.0, "longitude": 127.0, "altitude": 700.0},
            5: {"latitude": 38.0, "longitude": 127.05, "altitude": 700.0},
            6: {"latitude": 38.0, "longitude": 127.1, "altitude": 700.0},
        },
        heading_map={4: 0.0, 5: 0.0, 6: 0.0},
        entry_aircraft_context_map=None,
        representative_entry=None,
        next_entry=None,
        template_map={},
        template_record_map={},
        now_ms=1,
        turn_radius_scale=1.0,
        emit=messages.append,
        planning_mode={"package_type": 2},
        branch_ownership_override=ownership,
        branch_ownership_source_input_id=source_input_id,
    )

    assert result is None  # The fake planner returns no path rows.
    assert planner_input_ids == [source_input_id]
    assert target_mission["inputMissionID"] == target_input_id
    assert any("LINE branch ownership rebound" in message for message in messages)


def test_type2_branch_uses_its_own_declared_coordinate_order() -> None:
    line_specs = [
        {
            "coordinateList": [
                {"latitude": 38.0, "longitude": 127.0, "altitude": 0.0},
                {"latitude": 38.1, "longitude": 127.1, "altitude": 0.0},
            ]
        },
        {
            "coordinateList": [
                {"latitude": 38.2, "longitude": 127.3, "altitude": 0.0},
                {"latitude": 38.1, "longitude": 127.2, "altitude": 0.0},
            ]
        },
    ]
    piece = SplitPiece(
        parent_order=1,
        mission_id=4,
        mission_type=1,
        piece_index=2,
        data={"branchIndex": 1},
        assigned_uav=5,
    )

    direction_xy, deployment_coords = _type2_branch_deployment_reference(
        piece,
        line_specs,
        {"package_type": 2},
    )

    assert len(direction_xy) == 2
    assert deployment_coords == line_specs[1]["coordinateList"]
    assert deployment_coords[0]["longitude"] == 127.3
    assert deployment_coords[-1]["longitude"] == 127.2


def test_non_type2_does_not_force_branch_input_direction() -> None:
    piece = SplitPiece(1, 4, 1, 1, {"branchIndex": 0}, 4)
    specs = [{"coordinateList": [{"latitude": 38.0, "longitude": 127.0}, {"latitude": 38.1, "longitude": 127.1}]}]

    assert _type2_branch_deployment_reference(piece, specs, {"package_type": 1}) == ([], [])
