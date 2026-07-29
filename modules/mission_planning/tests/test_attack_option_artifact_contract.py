"""Regression contracts for publishing attack and attack-exclusion options.

An attack request may generate an exclusion alternative in parallel, but the
attack option itself is valid only when its referenced artifact graph contains
at least one executable LAH attack waypoint.  In particular, target-bearing
hold missions (type 9) are provenance/support rows and must never be mistaken
for a generated attack mission (type 2).
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path
from typing import Any

from modules.decision_support.core.option_processing import OptionPayloadBuilder
from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
    evaluate_candidate_attack_continuity,
)
from modules.mission_planning.runtime.validation.attack_option_publication import (
    resolve_post_attack_option_indices,
)


class _DbPaths:
    def __init__(self, root: Path) -> None:
        self.root = root

    def get_db_subpath(self, *parts: str) -> Path:
        return self.root.joinpath(*parts)


def _mission_plan(*aircraft_packages: tuple[int, int]) -> dict[str, Any]:
    return {
        "missionPlanID": 7_000_009,
        "aircraftList": [
            {
                "aircraftID": aircraft_id,
                "individualMissionPackageID": package_id,
            }
            for aircraft_id, package_id in aircraft_packages
        ],
    }


def _attack_imp(
    package_id: int,
    mission_id: int,
    path_id: int,
    target_id: int,
) -> dict[str, Any]:
    return {
        "individualMissionPackageID": package_id,
        "individualMissionList": [
            {
                "individualMissionID": mission_id,
                "pathID": path_id,
                "isDone": False,
                "individualMissionInfo": {
                    "individualMissionType": 2,
                    "targetID": target_id,
                },
            }
        ],
    }


def _hold_imp(
    package_id: int,
    mission_id: int,
    path_id: int,
    target_id: int,
) -> dict[str, Any]:
    return {
        "individualMissionPackageID": package_id,
        "individualMissionList": [
            {
                "individualMissionID": mission_id,
                "pathID": path_id,
                "isDone": False,
                "individualMissionInfo": {
                    "individualMissionType": 9,
                    "targetID": target_id,
                },
            }
        ],
    }


def _attack_path(
    path_id: int,
    aircraft_id: int,
    mission_id: int,
    waypoint_id: int,
    target_id: int,
) -> dict[str, Any]:
    return {
        "pathID": path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": mission_id,
        "lahWaypointList": [
            {
                "waypointID": waypoint_id,
                "nextWaypointID": 0,
                "isDone": False,
                "coordinate": {
                    "latitude": 38.0,
                    "longitude": 127.0,
                    "altitude": 500,
                },
                "attack": {"targetID": target_id, "weaponType": 2},
            }
        ],
    }


def _hold_path(
    path_id: int,
    aircraft_id: int,
    mission_id: int,
    waypoint_id: int,
) -> dict[str, Any]:
    return {
        "pathID": path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": mission_id,
        "lahWaypointList": [
            {
                "waypointID": waypoint_id,
                "nextWaypointID": 0,
                "isDone": False,
                "coordinate": {
                    "latitude": 38.0,
                    "longitude": 127.0,
                    "altitude": 500,
                },
                "loiter": {"duration": 5},
            }
        ],
    }


def _write_payload(root: Path, kind: str, artifact_id: int, payload: dict[str, Any]) -> None:
    path = root / kind / f"{artifact_id}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_two_aircraft_attack_graph_contains_both_executable_attacks() -> None:
    plan = _mission_plan((2, 8_000_058), (3, 8_000_043))
    imps = [
        _attack_imp(8_000_058, 9_000_283, 2_000_040, 9),
        _attack_imp(8_000_043, 9_000_213, 3_000_030, 8),
    ]
    paths = [
        _attack_path(2_000_040, 2, 9_000_283, 13_614, 9),
        _attack_path(3_000_030, 3, 9_000_213, 12_896, 8),
    ]

    rows, errors = collect_lah_attack_rows(
        plan,
        individual_mission_plans=imps,
        flight_paths=paths,
    )
    expected_pairs = attack._expected_attack_pairs_from_manned_sequences(
        {
            2: [{"target_id": 9}],
            3: [{"target_id": 8}],
        }
    )
    candidate_pairs = {
        (int(row["aircraftID"]), int(row["targetID"])) for row in rows
    }

    assert errors == []
    assert candidate_pairs == {(2, 9), (3, 8)}
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs=expected_pairs,
        candidate_pairs=candidate_pairs,
    )
    assert decision["ok"] is True
    assert decision["successfulNewPairs"] == [(2, 9), (3, 8)]


def test_exclusion_only_hold_graph_cannot_pass_as_an_attack_option() -> None:
    """The exact bad state: two assignments, but no type-2/attack waypoint."""

    plan = _mission_plan((2, 8_000_058), (3, 8_000_043))
    imps = [
        _hold_imp(8_000_058, 9_000_283, 2_000_040, 9),
        _hold_imp(8_000_043, 9_000_213, 3_000_030, 8),
    ]
    paths = [
        _hold_path(2_000_040, 2, 9_000_283, 13_614),
        _hold_path(3_000_030, 3, 9_000_213, 12_896),
    ]

    rows, errors = collect_lah_attack_rows(
        plan,
        individual_mission_plans=imps,
        flight_paths=paths,
    )
    expected_pairs = attack._expected_attack_pairs_from_manned_sequences(
        {
            2: [{"target_id": 9}],
            3: [{"target_id": 8}],
        }
    )
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs=expected_pairs,
        candidate_pairs=set(),
        scan_errors=errors,
    )

    assert rows == []
    assert errors == []
    assert decision["ok"] is False
    assert decision["hasAttackResult"] is False
    assert decision["allNewUnengageable"] is True
    assert decision["deferredNewPairs"] == [(2, 9), (3, 8)]


def test_failed_attack_can_never_publish_its_exclusion_alternative_alone() -> None:
    keep_indices, suppressed_exclusion_indices = resolve_post_attack_option_indices(
        option_count=2,
        attack_option_indices=[0],
        attack_exclusion_option_indices=[1],
        attack_plan_materialized=False,
    )

    assert keep_indices == []
    assert suppressed_exclusion_indices == {1}


def test_successful_attack_keeps_its_exclusion_alternative() -> None:
    keep_indices, suppressed_exclusion_indices = resolve_post_attack_option_indices(
        option_count=2,
        attack_option_indices=[0],
        attack_exclusion_option_indices=[1],
        attack_plan_materialized=True,
    )

    assert keep_indices == [1]
    assert suppressed_exclusion_indices == set()


def test_explicit_exclusion_only_request_remains_publishable() -> None:
    keep_indices, suppressed_exclusion_indices = resolve_post_attack_option_indices(
        option_count=1,
        attack_option_indices=[],
        attack_exclusion_option_indices=[0],
        attack_plan_materialized=False,
    )

    assert keep_indices == [0]
    assert suppressed_exclusion_indices == set()


def test_gui_applies_pair_guard_to_the_materialized_attack_result() -> None:
    gui_source = (
        Path(__file__).resolve().parents[1] / "mission_planning_gui.py"
    ).read_text(encoding="utf-8")

    guard_call_at = gui_source.index("resolve_post_attack_option_indices(")
    materialized_at = gui_source.index(
        "attack_plan_materialized=bool(attack_summary_info)",
        guard_call_at,
    )
    suppress_at = gui_source.index(
        "paired_attack_plan_not_materialized",
        materialized_at,
    )

    assert guard_call_at < materialized_at < suppress_at


def test_attack_graph_guard_runs_before_mission_plan_publication() -> None:
    """A hold-only candidate must be rejected before its 0301 root is written."""

    source = inspect.getsource(attack._apply_attack_plan_overrides)

    guard_at = source.index("continuity_decision = evaluate_candidate_attack_continuity(")
    reject_at = source.index("if not continuity_ok:", guard_at)
    publish_at = source.index("_write_json_file(plan_dest, new_plan_data)")

    assert guard_at < reject_at < publish_at


def test_0701_reports_both_attack_targets_but_none_for_exclusion_holds(
    tmp_path: Path,
) -> None:
    attack_plan = _mission_plan((2, 8_000_058), (3, 8_000_043))
    attack_plan["missionPlanID"] = 7_000_009
    exclusion_plan = _mission_plan((2, 8_000_068), (3, 8_000_069))
    exclusion_plan["missionPlanID"] = 7_000_010

    for kind, artifact_id, payload in (
        ("MissionPlan", 7_000_009, attack_plan),
        ("IndividualMissionPlan", 8_000_058, _attack_imp(8_000_058, 9_000_283, 2_000_040, 9)),
        ("IndividualMissionPlan", 8_000_043, _attack_imp(8_000_043, 9_000_213, 3_000_030, 8)),
        ("MissionPlan", 7_000_010, exclusion_plan),
        ("IndividualMissionPlan", 8_000_068, _hold_imp(8_000_068, 9_000_383, 2_000_050, 9)),
        ("IndividualMissionPlan", 8_000_069, _hold_imp(8_000_069, 9_000_313, 3_000_040, 8)),
    ):
        _write_payload(tmp_path, kind, artifact_id, payload)

    option_list = OptionPayloadBuilder(_DbPaths(tmp_path)).build_option_list(
        [
            {
                "optionID": 5,
                "optionName": 2,
                "missionPlanID": 7_000_009,
            },
            {
                "optionID": 6,
                "optionName": 3,
                "missionPlanID": 7_000_010,
            },
        ]
    )

    assert option_list[0]["optionName"] == 2
    assert option_list[0]["targetIDListN"] == 2
    assert option_list[0]["targetIDList"] == [
        {"targetID": 9},
        {"targetID": 8},
    ]
    assert option_list[1]["optionName"] == 3
    assert option_list[1]["targetIDListN"] == 0
    assert option_list[1]["targetIDList"] == []
