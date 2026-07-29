from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
)


class _FakeAttackIdReservation:
    def __init__(self) -> None:
        self._path_ids = iter([2003])
        self._individual_ids = iter([203])
        self._waypoint_ids = iter([2300, 2301, 2302, 2303])

    def next_path(self, aircraft_id: int) -> int:
        assert aircraft_id == 2
        return next(self._path_ids)

    def next_individual(self) -> int:
        return next(self._individual_ids)

    def next_waypoint(self) -> int:
        return next(self._waypoint_ids)


def _mission(
    mission_id: int,
    path_id: int,
    *,
    target_id: int | None = None,
    input_id: int = 4,
) -> dict:
    return {
        "individualMissionID": mission_id,
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 1,
            "inputMissionID": input_id,
            "priorMissionID": 3,
        },
        "individualMissionInfo": {
            "individualMissionType": 2 if target_id is not None else 1,
            "targetID": target_id,
            "coordinateList": [],
        },
        "pathID": path_id,
    }


def _waypoint(
    waypoint_id: int,
    *,
    latitude: float,
    longitude: float,
    altitude: int,
    target_id: int | None = None,
) -> dict:
    waypoint = {
        "waypointID": waypoint_id,
        "nextWaypointID": 0,
        "isDone": False,
        "coordinate": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        },
        "velocity": {"speed": 60},
    }
    if target_id is not None:
        waypoint["attack"] = {"targetID": target_id, "weaponType": 2}
    return waypoint


def _stable_attack_identity(row: dict) -> tuple[int, int, int, int, int]:
    return (
        int(row["aircraftID"]),
        int(row["individualMissionID"]),
        int(row["pathID"]),
        int(row["waypointID"]),
        int(row["targetID"]),
    )


def test_two_committed_attacks_select_one_busy_lah_for_third_append() -> None:
    committed = [
        {"aircraftID": 2, "targetID": 10, "weaponType": 2},
        {"aircraftID": 3, "targetID": 11, "weaponType": 2},
    ]
    candidates = [
        {
            "aircraft_id": 2,
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 500},
            "weapon_inventory": {"type1": 0, "type2": 2, "type3": 0},
        },
        {
            "aircraft_id": 3,
            "coordinate": {"latitude": 38.1, "longitude": 127.1, "altitude": 500},
            "weapon_inventory": {"type1": 0, "type2": 2, "type3": 0},
        },
    ]

    selected, committed_row, reason = (
        attack_pipeline._select_incremental_attack_append_candidate(
            committed,
            [
                {
                    "target_id": 12,
                    "target_type": 1,
                    "coordinate": {
                        "latitude": 38.001,
                        "longitude": 127.001,
                        "altitude": 450,
                    },
                }
            ],
            candidates,
        )
    )

    assert reason == "ok"
    assert selected is not None
    assert committed_row is not None
    assert selected["aircraft_id"] == 2
    # One type-2 round remains after reserving the unfinished target-10 shot.
    assert selected["weapon_inventory"] == {"type1": 0, "type2": 1, "type3": 0}
    assert committed_row["targetID"] == 10


def test_append_hide_certification_evaluates_uav_los_at_hide_altitude(
    monkeypatch,
) -> None:
    hide = {"latitude": 38.0, "longitude": 127.0, "altitude": 527}
    los_calls: list[dict] = []

    monkeypatch.setattr(
        attack_pipeline,
        "_attack_los_resource_dir",
        lambda: "resource",
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_hide_point_masked_from_every_enemy",
        lambda *_args, **_kwargs: 3,
    )

    def _visible_uav_los(**kwargs):
        los_calls.append(dict(kwargs))
        return {"visible": True, "evaluated": True, "reason": "VISIBLE"}

    monkeypatch.setattr(attack_pipeline, "evaluate_regional_los", _visible_uav_los)

    certificate = attack_pipeline._certify_incremental_append_hide_endpoint(
        hide,
        {
            "enemy_contact": {
                "enemy_targets": [{"coordinate": hide}],
                "uav_states": [
                    {
                        "aircraft_id": 4,
                        "coordinate": {
                            "latitude": 38.001,
                            "longitude": 127.001,
                            "altitude": 1500,
                        },
                    }
                ],
            }
        },
        emit=lambda _message: None,
    )

    assert certificate is not None
    assert certificate["certified"] is True
    assert certificate["uavLinkCount"] == 1
    assert len(los_calls) == 1
    assert los_calls[0]["target_altitude_m"] == 527.0


def test_append_relocates_when_inherited_endpoint_loses_uav_los(
    monkeypatch,
) -> None:
    origin = {"latitude": 38.0, "longitude": 127.0, "altitude": 500}
    midpoint = {"latitude": 38.001, "longitude": 127.001, "altitude": 480}
    replacement = {"latitude": 38.002, "longitude": 127.002, "altitude": 470}
    certification_calls: list[dict] = []

    def _certify(coord, *_args, **_kwargs):
        certification_calls.append(dict(coord))
        if len(certification_calls) == 1:
            return None
        return {
            "certified": True,
            "enemyCheckedCount": 3,
            "uavLinkCount": 1,
            "requiredUavLinks": 1,
        }

    monkeypatch.setattr(
        attack_pipeline,
        "_certify_incremental_append_hide_endpoint",
        _certify,
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: {
            "applied": True,
            "endpoint": replacement,
            "routeWaypoints": [origin, midpoint, replacement],
        },
    )

    hide, certificate, route = (
        attack_pipeline._resolve_incremental_append_hide_endpoint(
            origin,
            {"aircraft_id": 3, "enemy_contact": {}},
            {"coordinate": {"latitude": 37.9, "longitude": 126.9, "altitude": 600}},
            emit=lambda _message: None,
        )
    )

    assert hide == replacement
    assert certificate["certified"] is True
    assert certificate["relocated"] is True
    assert route == [origin, midpoint, replacement]
    assert certification_calls == [origin, replacement]


def test_append_keeps_a_degraded_route_when_relocation_is_unavailable(
    monkeypatch,
) -> None:
    origin = {"latitude": 38.0, "longitude": 127.0, "altitude": 500}
    monkeypatch.setattr(
        attack_pipeline,
        "_certify_incremental_append_hide_endpoint",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: None,
    )

    hide, certificate, route = (
        attack_pipeline._resolve_incremental_append_hide_endpoint(
            origin,
            {"aircraft_id": 3, "enemy_contact": {}},
            {"coordinate": origin},
            emit=lambda _message: None,
        )
    )

    assert hide == origin
    assert certificate == {
        "certified": False,
        "relocated": False,
        "degradedDirect": True,
        "reason": "no_certified_append_hide",
    }
    assert route == [origin]


def test_append_builder_preserves_two_committed_graphs_and_adds_only_one_attack(
    tmp_path: Path,
    monkeypatch,
) -> None:
    lah2_imp = {
        "individualMissionPackageID": 20,
        "timestamp": 100,
        "individualMissionList": [
            _mission(101, 1001, input_id=3),
            _mission(201, 2001, target_id=10),
            {**_mission(202, 2002), "postAttackResume": True},
        ],
    }
    lah3_imp = {
        "individualMissionPackageID": 30,
        "individualMissionList": [_mission(301, 3001, target_id=11)],
    }
    original_lah2_missions = deepcopy(lah2_imp["individualMissionList"])
    hide = {"latitude": 38.0, "longitude": 127.0, "altitude": 500}
    lah2_committed_path = {
        "pathID": 2001,
        "aircraftID": 2,
        "individualMissionID": 201,
        "Source": "Manned",
        "lahWaypointList": [
            _waypoint(2100, **hide),
            _waypoint(
                2101,
                latitude=38.0001,
                longitude=127.0001,
                altitude=510,
                target_id=10,
            ),
            _waypoint(2102, **hide),
        ],
    }
    lah3_committed_path = {
        "pathID": 3001,
        "aircraftID": 3,
        "individualMissionID": 301,
        "lahWaypointList": [
            _waypoint(
                3101,
                latitude=38.01,
                longitude=127.01,
                altitude=510,
                target_id=11,
            )
        ],
    }

    def _db_path(kind: str, filename: str | None = None) -> Path:
        directory = tmp_path / kind
        directory.mkdir(parents=True, exist_ok=True)
        return directory / filename if filename else directory

    monkeypatch.setattr(attack_pipeline.db_paths, "get_db_subpath", _db_path)
    monkeypatch.setattr(
        attack_pipeline,
        "read_json_cached",
        lambda *_args, **_kwargs: deepcopy(lah2_committed_path),
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_certify_incremental_append_hide_endpoint",
        lambda *_args, **_kwargs: {
            "enemyCheckedCount": 3,
            "uavLinkCount": 2,
            "requiredUavLinks": 1,
        },
    )
    monkeypatch.setattr(
        attack_pipeline,
        "_attack_coordinate_at_hide_endpoint",
        lambda *_args, **_kwargs: {
            "latitude": 38.0,
            "longitude": 127.0,
            "altitude": 520,
            "attack_point_at_hide_endpoint": True,
        },
    )

    monkeypatch.setattr(
        attack_pipeline,
        "_validate_generated_artifact_write_entries",
        lambda **_kwargs: {},
    )

    committed_row = {
        "aircraftID": 2,
        "individualMissionPackageID": 20,
        "individualMissionID": 201,
        "pathID": 2001,
        "waypointID": 2101,
        "targetID": 10,
        "weaponType": 2,
        "missionIndex": 1,
        "waypointIndex": 1,
    }
    result = attack_pipeline._build_lah_incremental_attack_append_package(
        descriptor={
            "label": "manned",
            "mode": "LAH_ATTACK_APPEND",
            "aircraft_id": 2,
            "committed_attack_row": committed_row,
            "enemy_contact": {
                "enemy_targets": [
                    {
                        "target_id": 12,
                        "coordinate": {
                            "latitude": 38.002,
                            "longitude": 127.002,
                            "altitude": 450,
                        },
                    }
                ],
                "uav_states": [],
            },
        },
        assigned_targets=[
            {
                "target_id": 12,
                "target_type": 1,
                "selected_weapon_type": 2,
                "coordinate": {
                    "latitude": 38.002,
                    "longitude": 127.002,
                    "altitude": 450,
                },
            }
        ],
        new_imp_id=21,
        imp_data=lah2_imp,
        ctx={},
        state={"weapon_inventory": {"type1": 0, "type2": 1, "type3": 0}},
        aircraft_id=2,
        artifacts=SimpleNamespace(source_plan_id=700000001),
        emit=lambda _message: None,
        now_ms=200,
        id_reservation=_FakeAttackIdReservation(),
        defer_write=True,
    )

    assert result is not None
    assert result["appendOnly"] is True
    assert lah2_imp["individualMissionPackageID"] == 20
    assert lah2_imp["timestamp"] == 100
    assert lah2_imp["individualMissionList"] == original_lah2_missions
    deferred = result["_deferredWriteEntries"]
    assert len(deferred) == 2
    imp_payload = next(
        payload for path, payload in deferred if Path(path).parent.name == "IndividualMissionPlan"
    )
    new_path = next(
        payload for path, payload in deferred if Path(path).parent.name == "FlightPath"
    )

    candidate_missions = imp_payload["individualMissionList"]
    assert [mission["individualMissionID"] for mission in candidate_missions] == [
        101,
        201,
        203,
        202,
    ]
    assert [
        mission for mission in candidate_missions if mission["individualMissionID"] != 203
    ] == original_lah2_missions
    assert [mission["pathID"] for mission in candidate_missions] == [1001, 2001, 2003, 2002]
    assert new_path["pathID"] == 2003
    assert new_path["aircraftID"] == 2
    assert new_path["individualMissionID"] == 203
    assert all(
        int(path_payload["pathID"]) not in {1001, 2001, 2002}
        for path, path_payload in deferred
        if Path(path).parent.name == "FlightPath"
    )

    source_plan = {
        "aircraftList": [
            {"aircraftID": 2, "individualMissionPackageID": 20},
            {"aircraftID": 3, "individualMissionPackageID": 30},
        ]
    }
    candidate_plan = {
        "aircraftList": [
            {"aircraftID": 2, "individualMissionPackageID": 21},
            {"aircraftID": 3, "individualMissionPackageID": 30},
        ]
    }
    source_rows, source_errors = collect_lah_attack_rows(
        source_plan,
        individual_mission_plans=[lah2_imp, lah3_imp],
        flight_paths=[lah2_committed_path, lah3_committed_path],
    )
    candidate_rows, candidate_errors = collect_lah_attack_rows(
        candidate_plan,
        individual_mission_plans=[imp_payload, lah3_imp],
        flight_paths=[lah2_committed_path, lah3_committed_path, new_path],
    )
    assert source_errors == []
    assert candidate_errors == []
    source_identities = {_stable_attack_identity(row) for row in source_rows}
    candidate_identities = {_stable_attack_identity(row) for row in candidate_rows}
    assert source_identities == {
        (2, 201, 2001, 2101, 10),
        (3, 301, 3001, 3101, 11),
    }
    assert source_identities.issubset(candidate_identities)
    assert candidate_identities - source_identities == {(2, 203, 2003, 2300, 12)}
    source_order = {
        _stable_attack_identity(row): (row["missionIndex"], row["waypointIndex"])
        for row in source_rows
    }
    candidate_order = {
        _stable_attack_identity(row): (row["missionIndex"], row["waypointIndex"])
        for row in candidate_rows
    }
    assert all(candidate_order[identity] == order for identity, order in source_order.items())
