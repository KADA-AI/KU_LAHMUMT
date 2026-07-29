from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from modules.mission_planning.mission_planning_gui import (
    MainWindow,
    _attack_pipeline_preserved_noop_status,
    _post_attack_empty_result_completion_policy,
)
from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline
from modules.mission_planning.replanning.triggers.post_attack import (
    pipeline as post_attack_pipeline,
)
from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _partition_committed_attacks_by_destroyed_targets,
    _partition_novel_attack_targets,
)
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
    compare_post_attack_pairs,
    evaluate_candidate_attack_continuity,
    missing_attack_identities,
)


def _attack_waypoint(waypoint_id: int, target_id: int) -> dict:
    return {
        "waypointID": waypoint_id,
        "isDone": False,
        "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 500},
        "attack": {"targetID": target_id, "weaponType": 2},
    }


def _imp(imp_id: int, mission_id: int, path_id: int, target_id: int) -> dict:
    return {
        "individualMissionPackageID": imp_id,
        "individualMissionList": [
            {
                "individualMissionID": mission_id,
                "isDone": False,
                "individualMissionInfo": {
                    "individualMissionType": 2,
                    "targetID": target_id,
                },
                "pathID": path_id,
            }
        ],
    }


def _path(path_id: int, aircraft_id: int, mission_id: int, waypoint_id: int, target_id: int) -> dict:
    return {
        "pathID": path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": mission_id,
        "lahWaypointList": [_attack_waypoint(waypoint_id, target_id)],
    }


def test_new_detection_keeps_committed_targets_first_and_deduplicates() -> None:
    committed = [
        {"aircraftID": 3, "targetID": 10},
        {"aircraftID": 3, "targetID": 14},
    ]
    requested = [
        {"target_id": 15, "watcher_id": 6},
        {"target_id": 14, "watcher_id": 5},
    ]

    novel, already_committed, deferred = _partition_novel_attack_targets(
        requested,
        committed,
    )

    assert [item["target_id"] for item in novel] == [15]
    assert [item["target_id"] for item in already_committed] == [14]
    assert deferred == []


def test_three_committed_targets_defer_a_fourth_without_replacing_one() -> None:
    committed = [
        {"aircraftID": 2, "targetID": 10},
        {"aircraftID": 3, "targetID": 14},
        {"aircraftID": 3, "targetID": 16},
    ]

    novel, already_committed, deferred = _partition_novel_attack_targets(
        [{"target_id": 15, "watcher_id": 6}],
        committed,
    )

    assert novel == []
    assert already_committed == []
    assert [item["target_id"] for item in deferred] == [15]


def test_destroyed_committed_attacks_do_not_consume_new_target_capacity() -> None:
    committed = [
        {"aircraftID": 2, "targetID": 9},
        {"aircraftID": 2, "targetID": 11},
        {"aircraftID": 3, "targetID": 8},
    ]
    target_entries = [
        {"target_id": 8, "is_destroyed": True},
        {"target_id": 9, "is_destroyed": True},
        {"target_id": 11, "is_destroyed": True},
        {"target_id": 12, "is_destroyed": False},
    ]

    live, stale = _partition_committed_attacks_by_destroyed_targets(
        committed,
        target_entries,
    )
    novel, already_committed, deferred = _partition_novel_attack_targets(
        [{"target_id": 12, "watcher_id": 5}],
        live,
    )

    assert live == []
    assert sorted((row["aircraftID"], row["targetID"]) for row in stale) == [
        (2, 9),
        (2, 11),
        (3, 8),
    ]
    assert [item["target_id"] for item in novel] == [12]
    assert already_committed == []
    assert deferred == []


@pytest.mark.parametrize(
    "status",
    [
        "preserved_existing_attacks",
        "deferred_until_attack_slot_free",
    ],
)
def test_gui_treats_preserved_attack_results_as_normal_noop(status: str) -> None:
    assert _attack_pipeline_preserved_noop_status({"status": status}) == status
    assert _attack_pipeline_preserved_noop_status({"status": status.upper()}) == status


@pytest.mark.parametrize("result_body", [None, {}, {"status": "ok"}, {"status": "failed"}])
def test_gui_does_not_hide_real_empty_attack_results(result_body: object) -> None:
    assert _attack_pipeline_preserved_noop_status(result_body) is None


def test_candidate_guard_commits_certified_subset_and_defers_only_failed_pair() -> None:
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs={(2, 10), (2, 11), (3, 12)},
        candidate_pairs={(3, 12)},
        certified_deferred_pairs={(2, 10), (2, 11)},
    )

    assert decision["ok"] is True
    assert decision["partialSuccess"] is True
    assert decision["successfulNewPairs"] == [(3, 12)]
    assert decision["deferredNewPairs"] == [(2, 10), (2, 11)]


def test_candidate_guard_rejects_hold_only_result_when_every_new_attack_failed() -> None:
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs={(2, 10), (3, 11)},
        candidate_pairs=set(),
        certified_deferred_pairs={(2, 10), (3, 11)},
    )

    assert decision["ok"] is False
    assert decision["structuralOk"] is True
    assert decision["allNewUnengageable"] is True


def test_candidate_guard_never_relaxes_existing_attack_identity_failures() -> None:
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs={(3, 12)},
        candidate_pairs={(3, 12)},
        missing_committed_identities=[(2, 20, 201, 2001, 2101, 10)],
    )

    assert decision["ok"] is False
    assert decision["structuralOk"] is False


def test_candidate_guard_rejects_unexplained_new_attack_loss() -> None:
    decision = evaluate_candidate_attack_continuity(
        expected_new_pairs={(2, 10), (3, 12)},
        candidate_pairs={(3, 12)},
        certified_deferred_pairs=set(),
    )

    assert decision["ok"] is False
    assert decision["uncertifiedMissingPairs"] == [(2, 10)]


def test_attack_replan_identity_guard_detects_owner_or_path_rebuild() -> None:
    source_plan = {
        "aircraftList": [
            {"aircraftID": 2, "individualMissionPackageID": 20},
            {"aircraftID": 3, "individualMissionPackageID": 30},
        ]
    }
    source_imps = [_imp(20, 201, 2001, 10), _imp(30, 301, 3001, 14)]
    source_paths = [_path(2001, 2, 201, 2101, 10), _path(3001, 3, 301, 3101, 14)]
    source_rows, source_errors = collect_lah_attack_rows(
        source_plan,
        individual_mission_plans=source_imps,
        flight_paths=source_paths,
    )

    # The target still exists, but moving it to another LAH/new path is exactly
    # the progress-resetting behavior the guard must reject.
    rebuilt_plan = {
        "aircraftList": [
            {"aircraftID": 2, "individualMissionPackageID": 21},
            {"aircraftID": 3, "individualMissionPackageID": 31},
        ]
    }
    rebuilt_imps = [_imp(21, 211, 2101, 14), _imp(31, 311, 3101, 15)]
    rebuilt_paths = [_path(2101, 2, 211, 2201, 14), _path(3101, 3, 311, 3201, 15)]
    rebuilt_rows, rebuilt_errors = collect_lah_attack_rows(
        rebuilt_plan,
        individual_mission_plans=rebuilt_imps,
        flight_paths=rebuilt_paths,
    )

    assert source_errors == []
    assert rebuilt_errors == []
    assert missing_attack_identities(source_rows, rebuilt_rows) == [
        (2, 20, 201, 2001, 2101, 10),
        (3, 30, 301, 3001, 3101, 14),
    ]


def test_post_attack_guard_allows_closed_target_removal_but_not_remaining_loss() -> None:
    source = [
        {"aircraftID": 2, "targetID": 11},
        {"aircraftID": 3, "targetID": 10},
        {"aircraftID": 3, "targetID": 14},
    ]
    correct_candidate = [
        {"aircraftID": 3, "targetID": 10},
        {"aircraftID": 3, "targetID": 14},
    ]
    missing_candidate = [{"aircraftID": 3, "targetID": 10}]

    assert compare_post_attack_pairs(
        source,
        correct_candidate,
        closed_target_ids={11},
    )["ok"] is True
    failed = compare_post_attack_pairs(
        source,
        missing_candidate,
        closed_target_ids={11},
    )
    assert failed["ok"] is False
    assert failed["missing"] == [(3, 0, 0, 0, 0, 14)]


def test_post_attack_guard_rejects_reset_of_remaining_attack_artifacts() -> None:
    source = [
        {
            "aircraftID": 3,
            "individualMissionPackageID": 30,
            "individualMissionID": 301,
            "pathID": 3001,
            "waypointID": 3101,
            "targetID": 10,
        },
        {
            "aircraftID": 3,
            "individualMissionPackageID": 30,
            "individualMissionID": 302,
            "pathID": 3002,
            "waypointID": 3201,
            "targetID": 14,
        },
    ]
    # Regenerating the queued attack mission/path/WP would reset execution
    # progress and must fail.  The append pipeline separately normalizes its
    # one explicitly declared replacement package shell before this check.
    candidate = [
        {
            "aircraftID": 3,
            "individualMissionPackageID": 31,
            "individualMissionID": 399,
            "pathID": 3999,
            "waypointID": 3991,
            "targetID": 14,
        }
    ]

    result = compare_post_attack_pairs(
        source,
        candidate,
        closed_target_ids={10},
    )

    assert result["ok"] is False
    assert result["missing"] == [(3, 30, 302, 3002, 3201, 14)]


def test_post_attack_guard_rejects_closed_attack_left_in_candidate_package() -> None:
    source = [
        {
            "aircraftID": 3,
            "individualMissionID": 301,
            "pathID": 3001,
            "waypointID": 3101,
            "targetID": 10,
        },
        {
            "aircraftID": 3,
            "individualMissionID": 302,
            "pathID": 3002,
            "waypointID": 3201,
            "targetID": 14,
        },
    ]
    candidate = [dict(row) for row in source]

    result = compare_post_attack_pairs(
        source,
        candidate,
        closed_target_ids={10},
    )

    assert result["ok"] is False
    assert result["staleClosed"] == [(3, 0, 301, 3001, 3101, 10)]


def test_attack_source_prefers_existing_applied_progress_plan(
    tmp_path: Path,
    monkeypatch,
) -> None:
    plan_dir = tmp_path / "MissionPlan"
    plan_dir.mkdir()
    (plan_dir / "700000002.json").write_text("{}", encoding="utf-8")

    monkeypatch.setattr(
        attack_pipeline,
        "_load_latest_mission_progress_plan_id",
        lambda: 700000002,
    )
    monkeypatch.setattr(
        attack_pipeline.db_paths,
        "get_db_subpath",
        lambda kind, filename=None: plan_dir / filename if filename else plan_dir,
    )

    assert attack_pipeline._resolve_attack_source_plan_id(
        {"currentMissionPlanID": 700000001},
        {"currentMissionPlanID": 700000001},
    ) == 700000002


def test_attack_source_falls_back_when_progress_plan_artifact_is_missing(
    tmp_path: Path,
    monkeypatch,
) -> None:
    plan_dir = tmp_path / "MissionPlan"
    plan_dir.mkdir()

    monkeypatch.setattr(
        attack_pipeline,
        "_load_latest_mission_progress_plan_id",
        lambda: 700000099,
    )
    monkeypatch.setattr(
        attack_pipeline.db_paths,
        "get_db_subpath",
        lambda kind, filename=None: plan_dir / filename if filename else plan_dir,
    )

    assert attack_pipeline._resolve_attack_source_plan_id(
        {"currentMissionPlanID": 700000003},
        {"currentMissionPlanID": 700000004},
    ) == 700000004


def test_completed_mission_and_attack_waypoint_are_not_live_commitments() -> None:
    plan = {
        "aircraftList": [
            {"aircraftID": 2, "individualMissionPackageID": 20},
        ]
    }

    completed_mission = _imp(20, 201, 2001, 10)
    completed_mission["individualMissionList"][0]["isDone"] = True
    rows, errors = collect_lah_attack_rows(
        plan,
        individual_mission_plans=[completed_mission],
        # No path override is intentional: a completed historical mission
        # must not try to load its retired path or report a scan error.
        flight_paths=[],
    )
    assert rows == []
    assert errors == []

    active_mission = _imp(20, 201, 2001, 10)
    completed_attack_path = _path(2001, 2, 201, 2101, 10)
    completed_attack_path["lahWaypointList"][0]["isDone"] = True
    rows, errors = collect_lah_attack_rows(
        plan,
        individual_mission_plans=[active_mission],
        flight_paths=[completed_attack_path],
    )
    assert rows == []
    assert errors == []


def test_post_attack_continuity_uses_every_confirmed_destroyed_target(
    monkeypatch,
) -> None:
    def _row(aircraft_id: int, target_id: int) -> dict[str, int]:
        return {
            "aircraftID": aircraft_id,
            "individualMissionPackageID": 800000000 + target_id,
            "individualMissionID": 900000000 + target_id,
            "pathID": (aircraft_id * 100000000) + target_id,
            "waypointID": 1000 + target_id,
            "targetID": target_id,
        }

    stale_closed = _row(2, 7)
    current_closed = _row(3, 8)
    survivor = _row(2, 10)
    source_rows = [stale_closed, current_closed, survivor]
    monkeypatch.setattr(
        post_attack_pipeline,
        "_known_destroyed_target_ids",
        lambda _target_id: {7, 8},
    )

    accepted = post_attack_pipeline._compare_post_attack_live_attacks(
        source_rows,
        [survivor],
        target_id=8,
    )
    assert accepted["ok"] is True
    assert accepted["closedTargetIDs"] == [7, 8]
    assert accepted["expected"] == accepted["actual"]

    stale_candidate = post_attack_pipeline._compare_post_attack_live_attacks(
        source_rows,
        [stale_closed, survivor],
        target_id=8,
    )
    assert stale_candidate["ok"] is False
    assert stale_candidate["staleClosed"] == [
        (
            stale_closed["aircraftID"],
            stale_closed["individualMissionPackageID"],
            stale_closed["individualMissionID"],
            stale_closed["pathID"],
            stale_closed["waypointID"],
            stale_closed["targetID"],
        )
    ]

    missing_survivor = post_attack_pipeline._compare_post_attack_live_attacks(
        source_rows,
        [],
        target_id=8,
    )
    assert missing_survivor["ok"] is False
    assert missing_survivor["missing"] == [
        (
            survivor["aircraftID"],
            survivor["individualMissionPackageID"],
            survivor["individualMissionID"],
            survivor["pathID"],
            survivor["waypointID"],
            survivor["targetID"],
        )
    ]


def test_post_attack_empty_invariant_failure_has_terminal_failure_policy() -> None:
    policy = _post_attack_empty_result_completion_policy(
        {
            "status": "skipped",
            "reason": "live_attack_continuity_invariant_failed",
        },
        [],
    )

    assert policy == {
        "kind": "failure",
        "reason": "live_attack_continuity_invariant_failed",
        "detail": (
            "공격 후 복귀 재계획 결과 없음 "
            "(live_attack_continuity_invariant_failed)"
        ),
    }


def test_post_attack_empty_benign_skip_keeps_noop_policy() -> None:
    policy = _post_attack_empty_result_completion_policy(
        {
            "status": "skipped",
            "reason": "rejoin_not_needed",
        },
        [],
    )

    assert policy == {
        "kind": "noop",
        "reason": "rejoin_not_needed",
        "detail": "협업 복귀 재계획 불필요",
    }


def test_post_attack_invariant_rejection_sends_terminal_failure_completion() -> None:
    failure_completions: list[str] = []
    emitted: list[str] = []

    class _Window:
        log_sig = SimpleNamespace(emit=emitted.append)

        @staticmethod
        def _should_use_post_attack_rejoin_pipeline(_ctx) -> bool:
            return True

        @staticmethod
        def _run_trigger_pipeline_with_source_cache(*_args, **_kwargs):
            return SimpleNamespace(
                summary={
                    "status": "skipped",
                    "reason": "live_attack_continuity_invariant_failed",
                },
                status="skipped",
                log_path="debug.json",
                plan_ids=[],
                option_names=[],
                plan_meta_map={},
            )

        @staticmethod
        def _push_replan_failure_completion(contents: str) -> bool:
            failure_completions.append(str(contents))
            return True

        @staticmethod
        def _push_replan_noop_completion(*_args, **_kwargs) -> bool:
            raise AssertionError("an invariant rejection is not a benign no-op")

        @staticmethod
        def _push_0001_notice(*_args, **_kwargs) -> None:
            raise AssertionError("an invariant rejection must use failure completion")

    handled, summary = MainWindow._try_run_post_attack_rejoin_pipeline(
        _Window(),
        {
            "replan_detail": {
                "trigger": "0402",
                "triggerType": "attackClosedDestroyed",
            }
        },
        "표적11 격파 후 복귀",
    )

    assert handled is True
    assert summary is not None
    assert summary["reason"] == "live_attack_continuity_invariant_failed"
    assert failure_completions == [
        (
            "공격 후 복귀 재계획 결과 없음 "
            "(live_attack_continuity_invariant_failed)"
        )
    ]
    assert any("실패 완료 신호" in message for message in emitted)


def test_post_attack_result_with_plan_needs_no_empty_completion() -> None:
    assert (
        _post_attack_empty_result_completion_policy(
            {
                "status": "success",
                "reason": "post_attack_rejoin_planned",
            },
            [700000018],
        )
        is None
    )
