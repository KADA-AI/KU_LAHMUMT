from __future__ import annotations

import json
import threading
from pathlib import Path

import pytest

from modules.mission_planning.app.message_handlers.replan_requests import (
    replan_delay_policy,
)
from modules.mission_planning.pipelines import next_collab_path_builder
from modules.mission_planning.replanning.triggers.next_collab import pipeline
from modules.mission_planning.runtime.validation import replan_payloads


def _single_lah_line_plan(width: object) -> tuple[dict, list[dict], list[dict]]:
    imp_id = 800_000_099
    mission_id = 900_000_099
    path_id = 100_000_099
    mission_plan = {
        "missionPlanID": 700_000_099,
        "aircraftList": [
            {"aircraftID": 1, "individualMissionPackageID": imp_id},
        ],
    }
    individual_mission_plans = [
        {
            "individualMissionPackageID": imp_id,
            "aircraftID": 1,
            "individualMissionList": [
                {
                    "aircraftID": 1,
                    "individualMissionID": mission_id,
                    "pathID": path_id,
                    "individualMissionInfo": {
                        "lineList": [
                            {
                                "width": width,
                                "coordinateList": [
                                    {"latitude": 37.0, "longitude": 127.0},
                                    {"latitude": 37.01, "longitude": 127.01},
                                ],
                            }
                        ]
                    },
                }
            ],
        }
    ]
    flight_paths = [
        {
            "pathID": path_id,
            "aircraftID": 1,
            "individualMissionID": mission_id,
            "lahWaypointList": [
                {"waypointID": 100_000_001, "nextWaypointID": 0},
            ],
        }
    ]
    return mission_plan, individual_mission_plans, flight_paths


@pytest.mark.parametrize("width", [0, 1000, 50_000])
def test_replan_validation_accepts_icd_uint_line_width(width: int) -> None:
    mission_plan, individual_mission_plans, flight_paths = _single_lah_line_plan(width)

    summary = replan_payloads.validate_replan_payloads(
        mission_plan=mission_plan,
        individual_mission_plans=individual_mission_plans,
        flight_paths=flight_paths,
        allow_existing_db_artifacts=False,
    )

    assert summary["individualMissions"] == 1


@pytest.mark.parametrize("width", [1000.0, True, "1000", -1, 50_001])
def test_replan_validation_rejects_non_icd_line_width(width: object) -> None:
    mission_plan, individual_mission_plans, flight_paths = _single_lah_line_plan(width)

    with pytest.raises(replan_payloads.ReplanValidationError, match=r"lineList\[0\]\.width"):
        replan_payloads.validate_replan_payloads(
            mission_plan=mission_plan,
            individual_mission_plans=individual_mission_plans,
            flight_paths=flight_paths,
            allow_existing_db_artifacts=False,
        )


def test_generated_artifact_validation_rejects_float_line_width() -> None:
    _mission_plan, individual_mission_plans, flight_paths = _single_lah_line_plan(1000.0)

    with pytest.raises(replan_payloads.ReplanValidationError, match=r"lineList\[0\]\.width"):
        replan_payloads.validate_generated_artifact_payloads(
            individual_mission_plans=individual_mission_plans,
            flight_paths=flight_paths,
            allow_existing_db_artifacts=False,
        )


def test_generated_artifact_validation_rejects_float_waypoint_altitude() -> None:
    _mission_plan, individual_mission_plans, flight_paths = _single_lah_line_plan(1000)
    flight_paths[0]["lahWaypointList"][0]["coordinate"] = {
        "latitude": 37.0,
        "longitude": 127.0,
        "altitude": 795.373,
    }

    with pytest.raises(
        replan_payloads.ReplanValidationError,
        match=r"coordinate\.altitude.*integer JSON",
    ):
        replan_payloads.validate_generated_artifact_payloads(
            individual_mission_plans=individual_mission_plans,
            flight_paths=flight_paths,
            allow_existing_db_artifacts=False,
        )


def test_next_collab_delay_policy_does_not_change_other_trigger_delays() -> None:
    next_collab = replan_delay_policy(
        {"replanDetail": {"trigger": "0803", "triggerType": "nextCollaborativeMission"}}
    )
    reexecute = replan_delay_policy(
        {
            "replanDetail": {
                "trigger": "0803",
                "triggerType": "collabReexecuteInputRefresh",
            }
        }
    )
    destroyed = replan_delay_policy(
        {"replanDetail": {"trigger": "0402", "triggerType": "attackClosedDestroyed"}}
    )
    generic = replan_delay_policy({"replanDetail": {"trigger": "0803"}})

    assert (next_collab.runtime_setting_key, next_collab.default_delay_ms) == (
        "replan_next_collab_schedule_delay_ms",
        1,
    )
    assert (reexecute.runtime_setting_key, reexecute.default_delay_ms) == (
        "replan_collab_reexecute_schedule_delay_ms",
        30,
    )
    assert (destroyed.runtime_setting_key, destroyed.default_delay_ms) == (None, 0)
    assert (generic.runtime_setting_key, generic.default_delay_ms) == (None, 100)


def test_fov_min_sep_cache_hits_and_invalidates_on_file_change(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    fov_db = tmp_path / "fov.csv"
    fov_db.write_text("fov,sep,width,vel\n3.2,900,1000,40\n", encoding="utf-8")
    loads: list[int] = []

    monkeypatch.setattr(next_collab_path_builder, "get_runtime_fov_db_path", lambda: fov_db)
    monkeypatch.setattr(
        next_collab_path_builder,
        "load_fov_db_rows",
        lambda: loads.append(1) or [{"fov": 3.2, "sep": 900.0}],
    )
    monkeypatch.setattr(
        next_collab_path_builder,
        "get_runtime_camera_adjust_fov_scale",
        lambda: 1.0,
    )
    with next_collab_path_builder._FOV_MIN_SEP_CACHE_LOCK:
        next_collab_path_builder._FOV_MIN_SEP_CACHE.clear()

    assert next_collab_path_builder._fov_db_min_sep_for_fov(3.2) == 900.0
    assert next_collab_path_builder._fov_db_min_sep_for_fov(3.2) == 900.0
    assert len(loads) == 1

    fov_db.write_text(
        "fov,sep,width,vel\n3.2,900,1000,40\n3.3,950,1050,41\n",
        encoding="utf-8",
    )
    assert next_collab_path_builder._fov_db_min_sep_for_fov(3.2) == 900.0
    assert len(loads) == 2


def test_line_parallel_build_keeps_input_and_waypoint_id_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    release = threading.Event()
    started = [threading.Event() for _ in range(3)]

    def build(index: int) -> dict:
        started[index].set()
        assert release.wait(timeout=2.0)
        return {
            "pathID": index + 1,
            "waypointList": [
                {
                    "waypointID": 0,
                    "nextWaypointID": 0,
                    "coordinate": {"latitude": float(index), "longitude": 127.0},
                }
            ],
        }

    monkeypatch.setattr(
        pipeline,
        "_next_collab_replacement_path_build_workers",
        lambda item_count, *, scope: min(3, item_count),
    )
    build_items = [
        (path_id, lambda index=index: build(index))
        for index, path_id in enumerate((400000101, 500000101, 600000101))
    ]

    timer = threading.Timer(
        0.01,
        lambda: release.set() if all(event.wait(timeout=2.0) for event in started) else None,
    )
    timer.start()
    try:
        generated = pipeline._build_replacement_flight_paths(
            build_items,
            emit=lambda _message: None,
            scope="LINE",
        )
    finally:
        release.set()
        timer.cancel()

    waypoint_ids = iter((501, 502, 503))
    pipeline._assign_replacement_waypoint_ids_in_order(
        generated_fp_by_path=generated,
        ordered_path_ids=[path_id for path_id, _build_fn in build_items],
        waypoint_id_provider=lambda: next(waypoint_ids),
        emit=lambda _message: None,
        scope="LINE",
    )

    assert list(generated) == [400000101, 500000101, 600000101]
    assert [
        generated[path_id]["waypointList"][0]["waypointID"]
        for path_id, _build_fn in build_items
    ] == [501, 502, 503]
    assert [
        generated[path_id]["waypointList"][0]["coordinate"]["latitude"]
        for path_id, _build_fn in build_items
    ] == [0.0, 1.0, 2.0]


def test_validator_resolves_each_existing_artifact_directory_once_and_keeps_link_checks(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    imp_dir = tmp_path / "IndividualMissionPlan"
    fp_dir = tmp_path / "FlightPath"
    imp_dir.mkdir()
    fp_dir.mkdir()

    aircraft_rows = []
    for aircraft_id in (1, 2):
        imp_id = 800000000 + aircraft_id
        mission_id = 900000000 + aircraft_id
        path_id = aircraft_id * 100000000 + 1
        aircraft_rows.append(
            {"aircraftID": aircraft_id, "individualMissionPackageID": imp_id}
        )
        (imp_dir / f"{imp_id}.json").write_text(
            json.dumps(
                {
                    "individualMissionPackageID": imp_id,
                    "aircraftID": aircraft_id,
                    "individualMissionList": [
                        {
                            "aircraftID": aircraft_id,
                            "individualMissionID": mission_id,
                            "pathID": path_id,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        (fp_dir / f"{path_id}.json").write_text(
            json.dumps(
                {
                    "pathID": path_id,
                    "aircraftID": aircraft_id,
                    "individualMissionID": mission_id,
                    "waypointList": [
                        {
                            "waypointID": aircraft_id * 10 + 1,
                            "nextWaypointID": aircraft_id * 10 + 2,
                        },
                        {
                            "waypointID": aircraft_id * 10 + 2,
                            "nextWaypointID": 0,
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )

    resolve_calls: list[str] = []

    def resolve(kind: str, filename: str | None = None) -> Path:
        resolve_calls.append(kind)
        base = tmp_path / kind
        return base / filename if filename else base

    monkeypatch.setattr(replan_payloads.db_paths, "get_db_subpath", resolve)
    mission_plan = {"missionPlanID": 700000001, "aircraftList": aircraft_rows}

    summary = replan_payloads.validate_replan_payloads(mission_plan=mission_plan)
    assert summary["flightPaths"] == 2
    assert resolve_calls.count("IndividualMissionPlan") == 1
    assert resolve_calls.count("FlightPath") == 1

    bad_path = fp_dir / "200000001.json"
    bad_payload = json.loads(bad_path.read_text(encoding="utf-8"))
    bad_payload["aircraftID"] = 3
    bad_path.write_text(json.dumps(bad_payload), encoding="utf-8")
    with pytest.raises(replan_payloads.ReplanValidationError, match="aircraft mismatch"):
        replan_payloads.validate_replan_payloads(mission_plan=mission_plan)
