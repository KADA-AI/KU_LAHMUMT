"""Type 2 각자도생: a LINE branch never migrates to another UAV on replan.

Missions 4 and 6 of a Type 2 package carry one lineList element per branch, and
each element belongs whole to one UAV for the entire branch span. The area
division planner has always pinned its components to the sticky ownership store;
the line planner assigned by prediction distance instead, so a replan handed a
UAV someone else's line. The Type-2 ownership guard then rejected the result and
failed the whole 다음 협업기저임무 request rather than replanning it.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.next_area_mode.planner_window import (
    NextAreaPlanningWindow,
)

# branch index -> owning UAV, as persisted by the initial split.
OWNERSHIP = {0: [4], 1: [6], 2: [5]}


class _Planner:
    """Only the two collaborators the assignment step actually uses."""

    _assign_split_result_by_branch_ownership = (
        NextAreaPlanningWindow._assign_split_result_by_branch_ownership
    )
    _assign_split_result_by_prediction_distance = (
        NextAreaPlanningWindow._assign_split_result_by_prediction_distance
    )

    def _uav_prediction_points_by_id(self) -> dict[int, tuple[float, float]]:
        # Deliberately ordered against the ownership map, so a prediction-based
        # assignment produces a visibly different answer.
        return {4: (0.0, 0.0), 5: (1.0, 0.0), 6: (2.0, 0.0)}

    def _piece_assignment_target_xy(self, piece: SplitPiece) -> tuple[float, float]:
        return (float(piece.piece_index), 0.0)


def _split_result(*, branch_ownership: dict[int, list[int]]) -> SplitRunResult:
    return SplitRunResult(
        uav_count=3,
        uav_ids=[4, 5, 6],
        pieces=[
            SplitPiece(
                parent_order=1,
                mission_id=4,
                mission_type=1,
                piece_index=index + 1,
                data={"branchIndex": index},
            )
            for index in range(3)
        ],
        branch_ownership=dict(branch_ownership),
    )


def _assign(**kwargs: Any) -> tuple[SplitRunResult, dict[str, Any]]:
    result = _split_result(**kwargs)
    report = _Planner()._assign_split_result_by_prediction_distance(result)
    return result, report


def test_each_branch_goes_to_its_stored_owner() -> None:
    result, report = _assign(branch_ownership=OWNERSHIP)

    assert report["branchOwnership"] is True
    assert report["assignedPieces"] == 3
    for piece in result.pieces:
        branch_index = piece.data["branchIndex"]
        assert piece.assigned_uav == OWNERSHIP[branch_index][0]


def test_the_ownership_guard_that_failed_the_replan_now_passes() -> None:
    """The exact check in the next_collab Type-2 guard."""

    result, _report = _assign(branch_ownership=OWNERSHIP)

    for piece in result.pieces:
        branch_index = piece.data.get("branchIndex")
        assert branch_index is not None
        assert piece.assigned_uav is not None
        assert piece.assigned_uav in OWNERSHIP[int(branch_index)]


def test_a_branch_with_no_stored_owner_is_left_unassigned() -> None:
    """Fail closed: a UAV must never take an unowned Type-2 branch."""

    result, report = _assign(branch_ownership={0: [4], 1: [6]})

    assert report["assignedPieces"] == 2
    unowned = [piece for piece in result.pieces if piece.data["branchIndex"] == 2]
    assert [piece.assigned_uav for piece in unowned] == [None]


def test_packages_without_branch_ownership_still_use_prediction_distance() -> None:
    """Only the 각자도생 packages are sticky; everything else is unchanged."""

    class _NoPredictions(_Planner):
        def _uav_prediction_points_by_id(self) -> dict[int, tuple[float, float]]:
            return {}

    result = _split_result(branch_ownership={})
    report = _NoPredictions()._assign_split_result_by_prediction_distance(result)

    # Fell through to the prediction path, which has nothing to work with here.
    assert not report.get("branchOwnership")
    assert report["assignedPieces"] == 0
    assert all(piece.assigned_uav is None for piece in result.pieces)
