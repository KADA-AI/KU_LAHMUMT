"""Excluding one target must not cancel another aircraft's engagement.

Observed: an exclusion raised for target 8 swept every target-bound branch off
all three manned aircraft, so LAH3's in-flight attack on target 7 vanished with
it. Target 7 stayed tracked by its UAV but was never shot, and the operator saw
the attack simply disappear.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _lah_attack_target_mission_indices,
)


def _mission(target_id: int, *, mission_type: int = 2, input_id: int = 2) -> dict[str, Any]:
    return {
        "relatedMission": {"inputMissionID": input_id},
        "individualMissionInfo": {
            "individualMissionType": mission_type,
            "targetID": target_id,
        },
    }


def _missions() -> list[dict[str, Any]]:
    return [
        _mission(0, mission_type=6),  # an ordinary sweep, never a target branch
        _mission(7),                  # LAH3 engaging target 7
        _mission(8),                  # the target being excluded
    ]


def test_a_sweep_drops_every_target_branch_by_default() -> None:
    indices = _lah_attack_target_mission_indices(
        _missions(), current_input_id=2, target_id=8, exclude_all_target_missions=True
    )

    assert indices == [1, 2]


def test_a_retained_target_survives_the_sweep() -> None:
    """Target 7 is still being engaged, so its branch must stay."""

    indices = _lah_attack_target_mission_indices(
        _missions(),
        current_input_id=2,
        target_id=8,
        exclude_all_target_missions=True,
        retained_target_ids=[7],
    )

    assert indices == [2]


def test_target_specific_sweep_never_removes_an_unrequested_target() -> None:
    indices = _lah_attack_target_mission_indices(
        _missions(),
        current_input_id=2,
        target_id=8,
        exclude_all_target_missions=True,
        excluded_target_ids={8},
    )

    assert indices == [2]


def test_retention_also_applies_to_the_per_target_path() -> None:
    indices = _lah_attack_target_mission_indices(
        _missions(), current_input_id=2, target_id=7, retained_target_ids=[7]
    )

    assert indices == []


def test_requested_targets_are_read_from_the_replan_detail() -> None:
    ctx = {"replan_detail": {"targetIDList": [{"targetID": 8}], "targetID": 8}}

    assert attack._attack_exclusion_requested_target_ids(ctx) == {8}


def test_a_detail_without_targets_means_exclude_everything() -> None:
    """A whole-package exclusion must still sweep the lot."""

    assert attack._attack_exclusion_requested_target_ids({"replan_detail": {}}) == set()


def test_only_live_engagements_are_retained(monkeypatch) -> None:
    monkeypatch.setattr(
        attack,
        "list_active_tracking_assignments",
        lambda: [
            {"active": True, "target_id": 7},
            {"active": False, "target_id": 9},
            {"active": True, "target_id": 8},
        ],
    )

    assert attack._actively_tracked_target_ids() == {7, 8}


def test_the_exclusion_target_is_not_retained(monkeypatch) -> None:
    """The whole point: exclude 8, keep 7."""

    monkeypatch.setattr(
        attack,
        "list_active_tracking_assignments",
        lambda: [{"active": True, "target_id": 7}, {"active": True, "target_id": 8}],
    )
    excluded = attack._attack_exclusion_requested_target_ids(
        {"replan_detail": {"targetID": 8}}
    )

    retained = sorted(
        target for target in attack._actively_tracked_target_ids() if target not in excluded
    )

    assert retained == [7]


def test_tracking_recovery_ignores_another_targets_assignment(monkeypatch) -> None:
    monkeypatch.setattr(
        attack,
        "get_tracking_assignment",
        lambda _aircraft_id: {
            "active": True,
            "target_id": 7,
            "attack_plan_id": 700000003,
            "source_plan_id": 700000001,
            "original_current_waypoint_id": 123,
        },
    )

    recovery = attack._resolve_attack_tracking_recovery(
        aircraft_id=5,
        source_plan_id=700000003,
        current_coord=None,
        emit=lambda _message: None,
        excluded_target_ids={8},
    )

    assert recovery is None


def test_tracking_recovery_detaches_the_requested_target(monkeypatch) -> None:
    monkeypatch.setattr(
        attack,
        "get_tracking_assignment",
        lambda _aircraft_id: {
            "active": True,
            "target_id": 8,
            "attack_plan_id": 700000003,
            "source_plan_id": 700000001,
            "original_current_waypoint_id": 123,
        },
    )

    recovery = attack._resolve_attack_tracking_recovery(
        aircraft_id=5,
        source_plan_id=700000003,
        current_coord=None,
        emit=lambda _message: None,
        excluded_target_ids={8},
    )

    assert recovery is not None
    assert recovery["split_waypoint_id"] == 123


def test_tracking_clear_is_limited_to_the_requested_target(monkeypatch) -> None:
    assignment = {
        "active": True,
        "target_id": 7,
        "attack_plan_id": 700000003,
    }
    monkeypatch.setattr(
        attack, "get_tracking_assignment", lambda _aircraft_id: dict(assignment)
    )
    cleared: list[int] = []
    monkeypatch.setattr(
        attack, "clear_tracking_assignment", lambda aircraft_id: cleared.append(aircraft_id)
    )

    assert attack._clear_attack_tracking_assignment_if_attached_to_plan(
        aircraft_id=5,
        source_plan_id=700000003,
        emit=lambda _message: None,
        excluded_target_ids={8},
    ) is False
    assert cleared == []

    assignment["target_id"] = 8
    assert attack._clear_attack_tracking_assignment_if_attached_to_plan(
        aircraft_id=5,
        source_plan_id=700000003,
        emit=lambda _message: None,
        excluded_target_ids={8},
    ) is True
    assert cleared == [5]
