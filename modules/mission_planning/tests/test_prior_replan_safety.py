from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from modules.mission_planning.mission_planning_gui import (
    MainWindow,
    _load_source_plan_package_ids,
    _pick_latest_package_json,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    AgentSnapshotSummary,
    PlanMissionArtifacts,
    _active_prior_aircraft_ids_for_source_plan,
    _apply_resume_path_trimming,
    _inject_prior_waypoint,
    _load_target_tracking_entry,
    _match_prior_post_rejoin_assignments,
    _plan_carries_prior_assignment,
    _rebase_prior_source_plan_to_latest_applied,
    _select_nearest_agent,
    _selected_prior_waypoint_reservation_count,
)


def _source_waypoints() -> list[dict]:
    return [
        {
            "waypointID": waypoint_id,
            "coordinate": {
                "latitude": 37.0 + waypoint_id * 0.001,
                "longitude": 127.0,
                "altitude": 100,
            },
            "speed": 30.0,
            "eta": waypoint_id,
            "nextWaypointID": waypoint_id + 1 if waypoint_id < 5 else 0,
        }
        for waypoint_id in range(1, 6)
    ]


def test_selected_prior_waypoint_reservation_covers_anchor_and_path_reassignment() -> None:
    flight_path = {"waypointList": _source_waypoints()}
    reserved_count = _selected_prior_waypoint_reservation_count(flight_path)
    assert reserved_count == 9

    reserved_ids = iter(range(1000, 1000 + reserved_count))
    consumed_ids: list[int] = []

    def allocate() -> int:
        value = next(reserved_ids)
        consumed_ids.append(value)
        return value

    # The prior approach and target consume the first two IDs before trimming.
    allocate()
    allocate()
    done, resume, _removed = _apply_resume_path_trimming(
        flight_path,
        artifacts=PlanMissionArtifacts(
            source_plan_id=700000001,
            aircraft_id=6,
            individual_mission_package_id=800000006,
            individual_mission_id=900000006,
            path_id=600000001,
            current_waypoint_id=2,
            previous_waypoint_id=1,
        ),
        sweep_progress=None,
        emit=lambda _message: None,
        current_coord={"latitude": 37.0015, "longitude": 127.0, "altitude": 100},
        waypoint_allocator=allocate,
    )

    assert len(done) == 2  # completed WP + visualization anchor
    assert len(resume) == 4
    assert len(consumed_ids) == reserved_count
    assert len({wp["waypointID"] for wp in done + resume}) == len(done) + len(resume)


def test_type2_prior_resume_speed_is_geometry_based_and_idempotent() -> None:
    current = {"latitude": 38.0, "longitude": 127.0, "altitude": 1000}
    carrier = {"latitude": 38.0, "longitude": 127.015, "altitude": 1000}
    source = {
        "pathID": 400000001,
        "waypointList": [
            {
                "waypointID": 10,
                "coordinate": dict(carrier),
                "speed": 40.0,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": [
                            {
                                "latitude": 38.0,
                                "longitude": 127.0,
                                "altitude": 0,
                            },
                            {
                                "latitude": 38.0,
                                "longitude": 127.6,
                                "altitude": 0,
                            },
                        ],
                        # Deliberately reproduce a speed already inflated by an
                        # earlier replan. The first pass must replace it.
                        "searchSpeed": 5000.0,
                    },
                },
                "isDone": False,
            }
        ],
    }

    def _artifacts(waypoint_id: int) -> PlanMissionArtifacts:
        return PlanMissionArtifacts(
            source_plan_id=700000001,
            aircraft_id=4,
            individual_mission_package_id=800000001,
            individual_mission_id=900000001,
            path_id=400000001,
            current_waypoint_id=int(waypoint_id),
            previous_waypoint_id=None,
        )

    first_ids = iter(range(10_000, 10_010))
    first_timing: dict = {}
    _done, first_resume, _removed = _apply_resume_path_trimming(
        source,
        artifacts=_artifacts(10),
        sweep_progress=None,
        emit=lambda _message: None,
        current_coord=current,
        waypoint_allocator=lambda: next(first_ids),
        timing=first_timing,
        allow_line_scan_sweep_point_trim=True,
        preserve_line_carrier_coordinates=True,
    )
    assert len(first_resume) == 1
    first_capture = first_resume[0]
    first_speed = first_capture["filmingProperty"]["lineSearch"]["searchSpeed"]
    assert first_capture["coordinate"]["latitude"] == carrier["latitude"]
    assert first_capture["coordinate"]["longitude"] == carrier["longitude"]
    assert first_speed < 5000.0
    assert first_timing["recompute_line_search_speed_from_geometry"]["synchronized"]
    assert first_timing["scale_line_search_speed"]["skipped"]

    second_ids = iter(range(20_000, 20_010))
    second_timing: dict = {}
    _done, second_resume, _removed = _apply_resume_path_trimming(
        {"pathID": 400000001, "waypointList": first_resume},
        artifacts=_artifacts(int(first_capture["waypointID"])),
        sweep_progress=None,
        emit=lambda _message: None,
        current_coord=current,
        waypoint_allocator=lambda: next(second_ids),
        timing=second_timing,
        allow_line_scan_sweep_point_trim=True,
        preserve_line_carrier_coordinates=True,
    )
    second_speed = second_resume[0]["filmingProperty"]["lineSearch"]["searchSpeed"]

    assert second_resume[0]["coordinate"]["latitude"] == carrier["latitude"]
    assert second_resume[0]["coordinate"]["longitude"] == carrier["longitude"]
    assert second_speed == first_speed
    assert second_timing["scale_line_search_speed"]["skipped"]


def test_exhausted_line_resume_becomes_one_five_second_loiter() -> None:
    capture_coords = [
        {"latitude": 37.0, "longitude": 127.0, "altitude": 0},
        {"latitude": 37.001, "longitude": 127.0, "altitude": 0},
    ]
    flight_path = {
        "waypointList": [
            {
                "waypointID": 1,
                "coordinate": {
                    "latitude": 37.0005,
                    "longitude": 127.001,
                    "altitude": 100,
                },
                "speed": 30.0,
                "nextWaypointID": 2,
                "waypointPassType": 1,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": capture_coords,
                        "searchSpeed": 20.0,
                    },
                },
                "isDone": True,
            },
            {
                "waypointID": 2,
                "coordinate": {
                    "latitude": 37.2,
                    "longitude": 127.2,
                    "altitude": 100,
                },
                "speed": 30.0,
                "nextWaypointID": 0,
                "waypointPassType": 1,
                "filmingProperty": {
                    "operationMode": 1,
                    "coordinateOrientation": {
                        "coordinate": capture_coords[-1],
                    },
                },
                "isDone": False,
            },
        ]
    }
    allocated_ids = iter(range(2000, 2010))
    hold_coord = {
        "latitude": 37.01,
        "longitude": 127.02,
        "altitude": 150,
    }

    _done, resume, _removed = _apply_resume_path_trimming(
        flight_path,
        artifacts=PlanMissionArtifacts(
            source_plan_id=700000001,
            aircraft_id=6,
            individual_mission_package_id=800000006,
            individual_mission_id=900000006,
            path_id=600000001,
            current_waypoint_id=2,
            previous_waypoint_id=1,
        ),
        sweep_progress=None,
        emit=lambda _message: None,
        current_coord={"latitude": 37.001, "longitude": 127.001, "altitude": 100},
        completion_hold_coord=hold_coord,
        waypoint_allocator=lambda: next(allocated_ids),
    )

    assert len(resume) == 1
    hold = resume[0]
    assert hold["coordinate"] == {
        "latitude": 37.01,
        "longitude": 127.02,
        "altitude": 150.0,
    }
    assert hold["waypointPassType"] == 2
    assert hold["nextWaypointID"] == 0
    assert hold["loiterProperty"] == {
        "radius": 180,
        "direction": 1,
        "time": 5,
        "speed": 30,
    }
    assert hold["filmingProperty"]["operationMode"] == 1
    assert "lineSearch" not in hold["filmingProperty"]
    assert hold["noCaptureCompletionLoiter"] is True


def test_type2_line_scan_progress_trims_executable_sweep_points() -> None:
    capture_coords = [
        {
            "latitude": 37.0 + index * 0.001,
            "longitude": 127.0,
            "altitude": 0,
        }
        for index in range(10)
    ]
    flight_path = {
        "waypointList": [
            {
                "waypointID": 10,
                "coordinate": {
                    "latitude": 37.0,
                    "longitude": 127.01,
                    "altitude": 100,
                },
                "speed": 30.0,
                "nextWaypointID": 0,
                "waypointPassType": 3,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": capture_coords,
                        "searchSpeed": 20.0,
                    },
                },
                "isDone": False,
            }
        ]
    }
    progress_entry = {
        "path_id": 600000001,
        "sweep_point_count": 10,
        "progress_points": 4,
        "progress_percent": 40,
        "buffer_points": 4,
        "progress_source": "line_scan",
        "line_scan": {
            "source": "line_scan_progress_monitor",
            "sweepPointCount": 10,
            "progressPoints": 4,
            "progressPercent": 40,
        },
    }

    _done, resume, _removed = _apply_resume_path_trimming(
        flight_path,
        artifacts=PlanMissionArtifacts(
            source_plan_id=700000001,
            aircraft_id=6,
            individual_mission_package_id=800000006,
            individual_mission_id=900000006,
            path_id=600000001,
            current_waypoint_id=10,
            previous_waypoint_id=None,
        ),
        sweep_progress={600000001: progress_entry},
        emit=lambda _message: None,
        current_coord={"latitude": 37.004, "longitude": 127.01, "altitude": 100},
        waypoint_allocator=iter(range(3000, 3010)).__next__,
        allow_line_scan_sweep_point_trim=True,
    )

    remaining_coords = resume[0]["filmingProperty"]["lineSearch"]["coordinateList"]
    assert len(remaining_coords) == 6
    assert remaining_coords[0]["latitude"] == capture_coords[4]["latitude"]
    assert all(not waypoint.get("isDone") for waypoint in resume)


def test_replan_line_suffix_uses_point_progress_when_elapsed_clock_is_inherited() -> None:
    capture_coords = [
        {
            "latitude": 38.076 + index * 0.0001,
            "longitude": 127.370,
            "altitude": 450,
        }
        for index in range(184)
    ]
    flight_path = {
        "waypointList": [
            {
                "waypointID": 39996,
                "coordinate": {
                    "latitude": 38.076,
                    "longitude": 127.370,
                    "altitude": 900,
                },
                "speed": 30.0,
                "nextWaypointID": 0,
                "waypointPassType": 3,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": capture_coords,
                        "searchSpeed": 20.0,
                    },
                },
                "isDone": False,
            }
        ]
    }
    # This mirrors the observed three-way LINE attack/rejoin record: only 18
    # of 184 points were photographed, but the mission-lineage clock was 159
    # of 161 seconds.  The five-second lookahead is already 23 points and must
    # not be expanded to all 184 points from the inherited elapsed clock.
    progress_entry = {
        "path_id": 600000187,
        "elapsed_seconds": 159,
        "remaining_seconds": 2,
        "planned_seconds": 161,
        "sweep_point_count": 184,
        "seconds_per_point": 0.875,
        "progress_percent": 10,
        "progress_points": 18,
        "buffer_seconds": 5,
        "buffer_percent": 12,
        "buffer_points": 23,
    }

    _done, resume, _removed = _apply_resume_path_trimming(
        flight_path,
        artifacts=PlanMissionArtifacts(
            source_plan_id=700000037,
            aircraft_id=6,
            individual_mission_package_id=800000238,
            individual_mission_id=900001072,
            path_id=600000187,
            current_waypoint_id=39996,
            previous_waypoint_id=None,
        ),
        sweep_progress={600000187: progress_entry},
        emit=lambda _message: None,
        current_coord={"latitude": 38.0778, "longitude": 127.370, "altitude": 900},
        waypoint_allocator=iter(range(41000, 41010)).__next__,
        preserve_line_carrier_coordinates=True,
    )

    assert len(resume) == 1
    assert resume[0].get("noCaptureCompletionLoiter") is not True
    remaining_coords = resume[0]["filmingProperty"]["lineSearch"]["coordinateList"]
    assert len(remaining_coords) == 161
    assert remaining_coords[0] == capture_coords[23]
    assert resume[0]["isDone"] is False


def test_type2_guard_resume_does_not_trim_unconfirmed_lookahead_points() -> None:
    capture_coords = [
        {
            "latitude": 38.07 + index * 0.0001,
            "longitude": 127.37,
            "altitude": 450,
        }
        for index in range(192)
    ]
    flight_path = {
        "waypointList": [
            {
                "waypointID": 40953,
                "coordinate": {
                    "latitude": 38.07,
                    "longitude": 127.37,
                    "altitude": 900,
                },
                "speed": 30.0,
                "nextWaypointID": 0,
                "waypointPassType": 3,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": capture_coords,
                        "searchSpeed": 20.0,
                    },
                },
                "isDone": False,
            }
        ]
    }
    progress_entry = {
        "path_id": 500000236,
        "sweep_point_count": 192,
        "progress_points": 60,
        "buffer_points": 74,
        "seconds_per_point": 0.875,
    }

    _done, resume, _removed = _apply_resume_path_trimming(
        flight_path,
        artifacts=PlanMissionArtifacts(
            source_plan_id=700000042,
            aircraft_id=5,
            individual_mission_package_id=800000261,
            individual_mission_id=900001206,
            path_id=500000236,
            current_waypoint_id=40953,
            previous_waypoint_id=None,
        ),
        sweep_progress={500000236: progress_entry},
        emit=lambda _message: None,
        current_coord={"latitude": 38.0706, "longitude": 127.37, "altitude": 900},
        waypoint_allocator=iter(range(42000, 42010)).__next__,
        allow_line_scan_sweep_point_trim=True,
        preserve_line_carrier_coordinates=True,
    )

    remaining_coords = resume[0]["filmingProperty"]["lineSearch"]["coordinateList"]
    assert len(remaining_coords) == 132
    assert remaining_coords[0] == capture_coords[60]
    assert resume[0]["isDone"] is False


def test_prior_pipeline_none_returns_terminal_failure_instead_of_legacy_fallback() -> None:
    messages: list[str] = []
    dummy = SimpleNamespace(
        log_sig=SimpleNamespace(emit=messages.append),
        _try_run_prior_post_rejoin_pipeline=lambda *_args, **_kwargs: (False, None),
        _run_trigger_pipeline_with_source_cache=lambda *_args, **_kwargs: None,
    )

    result = MainWindow._try_run_prior_mission_pipeline(
        dummy,
        {"replan_level": 4, "replan_detail": {"priorMissionID": 1}},
        "선행임무:좌표지정",
    )

    assert result == {
        "status": "failed",
        "reason": "prior_pipeline_returned_none",
        "mode": "prior",
    }
    assert any("fallback blocked" in message for message in messages)


def test_package_ids_are_inherited_from_source_plan_before_latest_snapshot(tmp_path: Path) -> None:
    plan_path = tmp_path / "700000001.json"
    plan_path.write_text(
        json.dumps(
            {
                "missionPlanID": 700000001,
                "inputMissionPackageID": 2,
                "missionReferencePackageID": 1,
            }
        ),
        encoding="utf-8",
    )

    with patch(
        "modules.mission_planning.mission_planning_gui.db_paths.get_db_subpath",
        return_value=plan_path,
    ):
        assert _load_source_plan_package_ids(700000001) == {
            "inputMissionPackageID": 2,
            "missionReferencePackageID": 1,
            "sourceMissionPlanID": 700000001,
        }

        messages: list[str] = []
        dummy = SimpleNamespace(log_sig=SimpleNamespace(emit=messages.append))
        ctx = {
            "replan_level": 4,
            "replan_detail": {"sourceMissionPlanID": 700000001},
        }
        with patch(
            "modules.mission_planning.mission_planning_gui.get_latest_package_id",
            side_effect=AssertionError("latest snapshot must not override source-plan package IDs"),
        ):
            MainWindow._ensure_ctx_package_ids(dummy, ctx, {})

    assert ctx["inputMissionPackageID"] == 2
    assert ctx["missionReferencePackageID"] == 1


def test_last_resort_package_file_selection_uses_highest_numeric_id(tmp_path: Path) -> None:
    (tmp_path / "1.json").write_text("{}", encoding="utf-8")
    (tmp_path / "2.json").write_text("{}", encoding="utf-8")

    assert _pick_latest_package_json(tmp_path) == tmp_path / "2.json"


def test_active_prior_aircraft_are_kept_out_of_follow_up_prior_assignment() -> None:
    assignments = [
        {"active": True, "aircraft_id": 4, "prior_plan_id": 700000009},
        {"active": True, "aircraft_id": 5, "prior_plan_id": 700000099},
        {"active": False, "aircraft_id": 6, "prior_plan_id": 700000010},
    ]
    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.resolve_plan_lineage_ids",
            return_value={700000010, 700000009, 700000001},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=assignments,
        ),
    ):
        assert _active_prior_aircraft_ids_for_source_plan(700000010) == {4}


def test_active_prior_assignment_edges_restore_lineage_without_run_logs() -> None:
    assignments = [
        {
            "active": True,
            "aircraft_id": 4,
            "source_plan_id": 700000001,
            "prior_plan_id": 700000009,
        },
        {
            "active": True,
            "aircraft_id": 5,
            "source_plan_id": 700000009,
            "prior_plan_id": 700000010,
        },
    ]
    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.resolve_plan_lineage_ids",
            return_value={700000010},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=assignments,
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._plan_carries_prior_assignment",
            return_value=False,
        ),
    ):
        assert _active_prior_aircraft_ids_for_source_plan(700000010) == {4, 5}


def test_current_plan_imp_can_confirm_active_prior_without_lineage_logs(tmp_path: Path) -> None:
    mission_plan_dir = tmp_path / "MissionPlan"
    imp_dir = tmp_path / "IndividualMissionPlan"
    mission_plan_dir.mkdir()
    imp_dir.mkdir()
    (mission_plan_dir / "700000020.json").write_text(
        json.dumps(
            {
                "missionPlanID": 700000020,
                "aircraftList": [
                    {"aircraftID": 4, "individualMissionPackageID": 800000004}
                ],
            }
        ),
        encoding="utf-8",
    )
    (imp_dir / "800000004.json").write_text(
        json.dumps(
            {
                "individualMissionPackageID": 800000004,
                "individualMissionList": [
                    {"individualMissionID": 900000004, "pathID": 500000004}
                ],
            }
        ),
        encoding="utf-8",
    )

    def resolve_path(kind: str, filename: str = "") -> Path:
        return tmp_path / kind / filename

    with patch(
        "modules.mission_planning.replanning.triggers.prior.pipeline.db_paths.get_db_subpath",
        side_effect=resolve_path,
    ):
        assert _plan_carries_prior_assignment(
            700000020,
            {
                "aircraft_id": 4,
                "prior_individual_mission_id": 900000004,
                "prior_path_id": 500000004,
            },
        )


def test_stale_queued_prior_source_rebases_only_to_applied_descendant() -> None:
    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_latest_mission_progress_plan_id",
            return_value=700000011,
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.resolve_plan_lineage_ids",
            return_value={700000011, 700000010},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=[],
        ),
    ):
        assert _rebase_prior_source_plan_to_latest_applied(700000010) == 700000011

    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_latest_mission_progress_plan_id",
            return_value=700000099,
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.resolve_plan_lineage_ids",
            return_value={700000099},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=[],
        ),
    ):
        assert _rebase_prior_source_plan_to_latest_applied(700000010) == 700000010


def test_stale_prior_source_rebases_through_active_assignment_edges_without_run_logs() -> None:
    assignments = [
        {
            "active": True,
            "aircraft_id": 4,
            "source_plan_id": 700000001,
            "prior_plan_id": 700000002,
        },
        {
            "active": True,
            "aircraft_id": 5,
            "source_plan_id": 700000002,
            "prior_plan_id": 700000003,
        },
    ]
    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_latest_mission_progress_plan_id",
            return_value=700000003,
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.resolve_plan_lineage_ids",
            return_value={700000003},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=assignments,
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._plan_carries_prior_assignment",
            return_value=False,
        ),
    ):
        assert _rebase_prior_source_plan_to_latest_applied(700000001) == 700000003


def test_prior_close_matching_uses_log_independent_active_aircraft_resolution() -> None:
    assignments = [
        {"active": True, "aircraft_id": 4, "prior_plan_id": 700000002},
        {"active": True, "aircraft_id": 5, "prior_plan_id": 700000003},
    ]
    with (
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._active_prior_aircraft_ids_for_source_plan",
            return_value={4, 5},
        ),
        patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.list_active_prior_assignments",
            return_value=assignments,
        ),
    ):
        matched = _match_prior_post_rejoin_assignments(
            current_plan_id=700000003,
            aircraft_id=4,
        )

    assert [row["aircraft_id"] for row in matched] == [4]


def test_nearest_prior_uav_selection_excludes_active_prior_aircraft() -> None:
    summaries = [
        AgentSnapshotSummary(4, 37.0000, 127.0000, 700.0, 10, 0.0, 1),
        AgentSnapshotSummary(5, 37.0010, 127.0000, 700.0, 20, 0.0, 1),
        AgentSnapshotSummary(6, 37.0100, 127.0000, 700.0, 30, 0.0, 1),
    ]

    selected, distance_m = _select_nearest_agent(
        37.0001,
        127.0000,
        summaries,
        excluded_aircraft_ids={4, 5},
    )

    assert selected is not None
    assert selected.aircraft_id == 6
    assert distance_m is not None and distance_m > 0


def test_target_prior_waypoint_preserves_target_id_for_auto_tracking() -> None:
    flight_path = {
        "waypointList": [
            {
                "waypointID": 10,
                "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 700},
                "nextWaypointID": 11,
            },
            {
                "waypointID": 11,
                "coordinate": {"latitude": 37.1, "longitude": 127.1, "altitude": 700},
                "nextWaypointID": 0,
            },
        ]
    }

    _removed, inserted = _inject_prior_waypoint(
        flight_path,
        current_waypoint_id=10,
        previous_waypoint_id=None,
        target_coord={"latitude": 37.05, "longitude": 127.05, "altitude": 700},
        new_waypoint_id=99,
        mission_type=2,
        target_tracking={"targetID": 157},
    )

    filming = inserted["filmingProperty"]
    assert filming["operationMode"] == 3
    assert filming["autoTracking"] == {"targetID": 157}


def test_target_tracking_lookup_keeps_watcher_and_coordinate(tmp_path: Path) -> None:
    target_info = tmp_path / "targetInfo.json"
    target_info.write_text(
        json.dumps(
            {
                "targetList": {
                    "157-6": {
                        "targetID": 157,
                        "targetInFrame": True,
                        "lastUpdated": 1234,
                        "coordinate": {
                            "latitude": 37.87944,
                            "longitude": 128.27643,
                            "altitude": 700,
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    with patch(
        "modules.mission_planning.replanning.triggers.prior.pipeline.db_paths.get_db_subpath",
        return_value=target_info,
    ):
        result = _load_target_tracking_entry(157)

    assert result is not None
    assert result["targetID"] == 157
    assert result["watcherID"] == 6
    assert result["coordinate"]["latitude"] == 37.87944
