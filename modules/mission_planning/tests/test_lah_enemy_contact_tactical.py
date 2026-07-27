from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from affine import Affine
import numpy as np
import pytest

from modules.common.regional_dem import regional_dem_path_for_coordinate
from modules.mission_planning.pipelines import lah_enemy_contact
from modules.mission_planning.replanning.triggers.attack import pipeline
from modules.monitoring.logic.dem_cover.hide_com import HideEndpointCandidate
from modules.monitoring.logic.dem_cover import hide_com_refine
from modules.monitoring.logic.dem_cover.hide_com_route import (
    RouteDynamics,
    plan_hide_communication_routes,
)


def _certified_plan() -> dict:
    return {
        "applied": True,
        "status": "green_valid",
        "etaS": 8.2,
        "endpoint": {
            "latitude": 37.001,
            "longitude": 127.001,
            "altitude": 550.25,
        },
        "routeWaypoints": [
            {
                "latitude": 37.0,
                "longitude": 127.0,
                "altitude": 800.125,
                "etaS": 0.0,
                "speedMps": 40.0,
                "distanceM": 0.0,
            },
            {
                "latitude": 37.001,
                "longitude": 127.001,
                "altitude": 550.25,
                "etaS": 8.2,
                "speedMps": 45.0,
                "distanceM": 140.0,
            },
        ],
    }


class _Reservation:
    def __init__(self) -> None:
        self._waypoint_id = 1000

    def next_paths(self, _aircraft_id: int, count: int):
        return tuple(201 + index for index in range(count))

    def next_individuals(self, count: int):
        return tuple(301 + index for index in range(count))

    def next_waypoint(self) -> int:
        self._waypoint_id += 1
        return self._waypoint_id

    def next_path(self, _aircraft_id: int) -> int:
        return 299

    def next_individual(self) -> int:
        return 399


def test_command_aircraft_is_never_an_attack_candidate(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int",
        lambda key, default: 1 if key == "command_aircraft_id" else default,
    )
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int_list",
        lambda _key, _default: [1, 2, 3],
    )

    assert pipeline._attack_manned_candidates() == (2, 3)


def test_missing_enemy_context_cannot_disable_tactical_cover(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int",
        lambda key, default: 1 if key == "tactical_cover_enabled" else default,
    )

    assert pipeline._lah_tactical_cover_required({"mode": "LAH_ATTACK"}) is True
    assert pipeline._lah_tactical_cover_required({"mode": "LAH_RELAY"}) is True
    assert pipeline._lah_tactical_cover_required({"mode": "LAH_HOLD_RESUME"}) is False

    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int",
        lambda key, default: 0 if key == "tactical_cover_enabled" else default,
    )
    assert pipeline._lah_tactical_cover_required({"mode": "LAH_ATTACK"}) is False


def test_mandatory_tactical_results_require_both_status_and_update() -> None:
    descriptors = [
        {"aircraft_id": 1, "mode": "LAH_RELAY"},
        {"aircraft_id": 2, "mode": "LAH_ATTACK"},
        {"aircraft_id": 4, "mode": "UAV_TRACK"},
    ]
    results = [
        {
            "aircraftID": 1,
            "mode": "LAH_RELAY",
            "status": "ok",
            "update": None,
        },
        {
            "aircraftID": 2,
            "mode": "LAH_ATTACK",
            "status": "ok",
            "update": {"individualMissionPackageID": 22},
        },
    ]

    failures = pipeline._mandatory_tactical_descriptor_failures(
        descriptors,
        results,
    )

    assert [item["aircraftID"] for item in failures] == [1]
    assert failures[0]["mode"] == "LAH_RELAY"


def test_tactical_wrapper_forwards_role_specific_link_requirements(monkeypatch) -> None:
    calls: list[dict] = []

    def fake_plan(*args, **kwargs):
        calls.append({"args": args, "kwargs": kwargs})
        return {"applied": False, "status": "no_route", "failureCodes": ["TEST"]}

    monkeypatch.setattr(lah_enemy_contact, "plan_enemy_contact_response", fake_plan)
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int",
        lambda key, default: {
            "tactical_cover_enabled": 1,
            "tactical_relay_min_uav_links": 3,
            "tactical_attacker_min_uav_links": 1,
            "tactical_relay_degraded_min_uav_links": 2,
        }.get(key, default),
    )
    descriptor = {
        "aircraft_id": 1,
        "enemy_contact": {
            "uav_states": [{"aircraft_id": value} for value in (4, 5, 6)],
            "enemy_coordinates": [{"latitude": 37.1, "longitude": 127.1}],
        },
    }
    state = {
        "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800},
        "speed": 40.0,
        "heading": 0.0,
    }

    pipeline._plan_lah_enemy_contact_response(descriptor, state, role="relay", emit=lambda _: None)
    pipeline._plan_lah_enemy_contact_response(descriptor, state, role="attacker", emit=lambda _: None)

    relay_calls = [row for row in calls if row["kwargs"]["role"] == "relay"]
    attacker_calls = [row for row in calls if row["kwargs"]["role"] == "attacker"]

    # Concealment outranks link count: the command aircraft asks for three links
    # first, then walks the requirement down rather than accepting exposure.
    assert [row["kwargs"]["min_uav_links"] for row in relay_calls] == [3, 2, 1]
    assert all(row["kwargs"]["degraded_min_uav_links"] == 2 for row in relay_calls)

    # An attacker already starts at the floor, so there is nothing to walk down.
    assert [row["kwargs"]["min_uav_links"] for row in attacker_calls] == [1]
    assert all(row["kwargs"]["degraded_min_uav_links"] is None for row in attacker_calls)


def test_the_link_ladder_stops_at_the_first_concealed_answer(monkeypatch) -> None:
    """Never trade away links that concealment did not actually need."""

    calls: list[dict] = []

    def fake_plan(*args, **kwargs):
        calls.append(kwargs)
        # Three links already conceals, so the ladder must not descend.
        return {"applied": True, "status": "green_valid"}

    monkeypatch.setattr(lah_enemy_contact, "plan_enemy_contact_response", fake_plan)
    monkeypatch.setattr(
        pipeline,
        "get_runtime_attack_int",
        lambda key, default: {
            "tactical_cover_enabled": 1,
            "tactical_relay_min_uav_links": 3,
            "tactical_relay_degraded_min_uav_links": 2,
        }.get(key, default),
    )
    descriptor = {
        "aircraft_id": 1,
        "enemy_contact": {
            "uav_states": [{"aircraft_id": value} for value in (4, 5, 6)],
            "enemy_coordinates": [{"latitude": 37.1, "longitude": 127.1}],
        },
    }
    state = {
        "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800},
        "speed": 40.0,
        "heading": 0.0,
    }

    pipeline._plan_lah_enemy_contact_response(descriptor, state, role="relay", emit=lambda _: None)

    assert [row["min_uav_links"] for row in calls] == [3]


def test_tactical_route_serialization_emits_icd_integer_altitude() -> None:
    waypoint_ids = iter((1, 2))

    waypoints = pipeline._build_lah_tactical_route_waypoints(
        template_wp=pipeline._default_lah_waypoint_template(),
        plan=_certified_plan(),
        waypoint_id_provider=lambda: next(waypoint_ids),
        terminal_hover_seconds=300,
    )

    altitudes = [item["coordinate"]["altitude"] for item in waypoints]
    assert altitudes == [800, 550]
    assert all(type(altitude) is int for altitude in altitudes)
    assert [item["eta"] for item in waypoints] == [0, 9]
    assert [item["nextWaypointID"] for item in waypoints] == [2, 0]
    assert [item["hovering"]["time"] for item in waypoints] == [0, 300]
    assert all(item["attack"] == {"targetID": 0, "weaponType": 0} for item in waypoints)
    # ICD ecf is per-leg fuel in litres, so the terminal is no longer a
    # fixed 1.0 marker; it just has to stay inside the field range.
    assert 0.0 <= waypoints[-1]["ecf"] <= 1000.0


def test_concealment_route_time_uses_constant_acceleration() -> None:
    left = SimpleNamespace(timestamp_s=0.0, speed_mps=0.0)
    right = SimpleNamespace(timestamp_s=10.0, speed_mps=20.0)

    # With constant 2 m/s^2 acceleration, 25% of the distance is reached at
    # 5 seconds, not the 2.5 seconds produced by linear time interpolation.
    assert lah_enemy_contact._constant_acceleration_leg_time(left, right, 0.25) == 5.0
    assert lah_enemy_contact._constant_acceleration_leg_time(left, right, 1.0) == 10.0


def test_concealment_route_time_handles_constant_deceleration() -> None:
    left = SimpleNamespace(timestamp_s=2.0, speed_mps=20.0)
    right = SimpleNamespace(timestamp_s=12.0, speed_mps=0.0)

    # The first 75% of distance is covered in the first half of a symmetric
    # constant-deceleration leg.
    assert lah_enemy_contact._constant_acceleration_leg_time(left, right, 0.75) == 7.0


def test_tactical_prelude_deduplicates_endpoint_and_offsets_attack_eta() -> None:
    prefix = pipeline._build_lah_tactical_route_waypoints(
        template_wp=pipeline._default_lah_waypoint_template(),
        plan=_certified_plan(),
        waypoint_id_provider=iter((1, 2)).__next__,
    )
    duplicate = pipeline._default_lah_waypoint_template()
    duplicate.update(
        {
            "waypointID": 3,
            "coordinate": {
                "latitude": 37.001,
                "longitude": 127.001,
                "altitude": 550.25,
            },
            "eta": 0,
        }
    )
    attack = pipeline._default_lah_waypoint_template()
    attack.update(
        {
            "waypointID": 4,
            "coordinate": {"latitude": 37.01, "longitude": 127.01, "altitude": 700},
            "eta": 20,
            "attack": {"targetID": 88, "weaponType": 2},
        }
    )

    combined = pipeline._prepend_lah_tactical_waypoints(prefix, [duplicate, attack])

    assert [item["waypointID"] for item in combined] == [1, 2, 4]
    assert [item["eta"] for item in combined] == [0, 9, 29]
    assert [item["nextWaypointID"] for item in combined] == [2, 4, 0]
    assert [item["attack"]["targetID"] for item in combined] == [0, 0, 88]


def test_relay_builder_keeps_type9_contract_and_uses_certified_route(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: _certified_plan(),
    )
    monkeypatch.setattr(
        pipeline,
        "_split_done_resume_lah_path",
        lambda *_args, **_kwargs: ([], [], None),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: ([], [], {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0}),
    )
    monkeypatch.setattr(pipeline, "_trim_lah_follow_up_paths_after_anchor", lambda **_kwargs: None)
    monkeypatch.setattr(pipeline, "_validate_generated_artifact_write_entries", lambda **_kwargs: None)
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(pipeline, "get_runtime_attack_int", lambda _key, default: default)
    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "coordinateList": [],
        },
    }
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [pipeline._default_lah_waypoint_template()],
    }

    result = pipeline._build_lah_hold_resume_package(
        descriptor={
            "mode": "LAH_RELAY",
            "label": "lah_relay_1",
            "aircraft_id": 1,
            "target_id": 88,
        },
        new_imp_id=500,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        ctx={"mission_ids": [77]},
        state={
            "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800},
            "speed": 40.0,
            "heading": 0.0,
        },
        aircraft_id=1,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_Reservation(),
        defer_write=True,
    )

    assert result is not None and result["relayMode"] is True
    assert result["hold"]["waypointCount"] == 2
    mission_info = imp_data["individualMissionList"][0]["individualMissionInfo"]
    assert mission_info["individualMissionType"] == 9
    assert mission_info["patternType"] == 12
    assert mission_info["targetID"] == 88
    hold_payload = next(
        payload
        for path, payload in result["_deferredWriteEntries"]
        if path.parent.name == "FlightPath" and int(payload.get("pathID")) == 201
    )
    hold_waypoints = hold_payload["lahWaypointList"]
    assert hold_waypoints[-1]["coordinate"]["altitude"] == 550
    # Only the terminal waypoint holds, and it holds for the fallback window:
    # this fixture has no strike geometry to size the wait from. The relay used
    # to sit here for a flat five minutes long after the strike was over.
    assert [item["hovering"]["time"] for item in hold_waypoints] == [
        0,
        pipeline._LAH_COVER_HOLD_DEFAULT_SECONDS,
    ]
    assert pipeline._LAH_COVER_HOLD_DEFAULT_SECONDS < 300
    assert all(item["attack"]["targetID"] == 0 for item in hold_waypoints)


@pytest.mark.parametrize(
    "planner_result",
    (
        None,
        {
            "applied": False,
            "status": "no_route",
            "failureCodes": ["NO_HIDE_THEN_RECONNECT_ROUTE"],
        },
    ),
)
def test_relay_no_route_holds_live_position_not_line_endpoint(
    monkeypatch,
    tmp_path: Path,
    planner_result,
) -> None:
    live = {"latitude": 37.2, "longitude": 127.2, "altitude": 420}
    line_end = {"latitude": 37.9, "longitude": 127.9, "altitude": 800}
    captured: dict[str, dict] = {}

    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: planner_result,
    )

    def fake_split(*_args, **kwargs):
        captured["split"] = kwargs
        return [], [], None

    def fake_trim(**kwargs):
        captured["trim"] = kwargs

    monkeypatch.setattr(pipeline, "_split_done_resume_lah_path", fake_split)
    monkeypatch.setattr(pipeline, "_trim_lah_follow_up_paths_after_anchor", fake_trim)
    monkeypatch.setattr(
        pipeline,
        "_build_lah_standby_hold_coordinate_from_path_end",
        lambda **_kwargs: (_ for _ in ()).throw(
            AssertionError("relay no-route must not use the LINE endpoint")
        ),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [],
            [],
            {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0},
        ),
    )
    monkeypatch.setattr(pipeline, "_validate_generated_artifact_write_entries", lambda **_kwargs: None)
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(pipeline, "get_runtime_attack_int", lambda _key, default: default)

    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 6,
            "patternType": 8,
            "coordinateList": [dict(line_end)],
        },
    }
    template = pipeline._default_lah_waypoint_template()
    template["coordinate"] = dict(line_end)
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [template],
    }

    result = pipeline._build_lah_hold_resume_package(
        descriptor={
            "mode": "LAH_RELAY",
            "label": "lah_relay_1",
            "aircraft_id": 1,
            "target_id": 88,
        },
        new_imp_id=500,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        ctx={"mission_ids": [77]},
        state={"coordinate": dict(live), "speed": 40.0, "heading": 0.0},
        aircraft_id=1,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_Reservation(),
        defer_write=True,
    )

    assert result is not None
    assert result["relayFallbackPolicy"] == "hold_live_current_no_certified_route"
    assert result["relayFallbackCertified"] is False
    hold_payload = next(
        payload
        for path, payload in result["_deferredWriteEntries"]
        if path.parent.name == "FlightPath" and int(payload.get("pathID")) == 201
    )
    assert hold_payload["lahWaypointList"][-1]["coordinate"] == live
    assert hold_payload["lahWaypointList"][-1]["coordinate"] != line_end
    assert captured["split"]["resume_trim_anchor_coord"] == live
    assert captured["trim"]["anchor_coord"] == live
    assert captured["trim"]["predict_anchor"] is False


def test_attacker_no_route_still_generates_the_precomputed_attack(
    monkeypatch,
    tmp_path: Path,
) -> None:
    live = {"latitude": 37.2, "longitude": 127.2, "altitude": 420}
    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: {
            "applied": False,
            "status": "no_route",
            "failureCodes": ["NO_HIDE_THEN_RECONNECT_ROUTE"],
        },
    )
    monkeypatch.setattr(
        pipeline,
        "_split_done_resume_lah_path",
        lambda *_args, **_kwargs: ([], [], None),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [],
            [],
            {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0},
        ),
    )
    monkeypatch.setattr(
        pipeline,
        "_trim_lah_follow_up_paths_after_anchor",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        pipeline,
        "_validate_generated_artifact_write_entries",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(pipeline, "get_runtime_attack_int", lambda _key, default: default)

    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "coordinateList": [],
        },
    }
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [pipeline._default_lah_waypoint_template()],
    }

    result = pipeline._build_lah_attack_package(
        descriptor={
            "mode": "LAH_ATTACK",
            "label": "manned",
            "aircraft_id": 2,
            "target_id": 88,
            "target_type": 1,
        },
        new_imp_id=501,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        attack_coord={"latitude": 37.5, "longitude": 127.5, "altitude": 700},
        ctx={"mission_ids": [77]},
        state={"coordinate": dict(live), "speed": 40.0, "heading": 0.0},
        aircraft_id=2,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_Reservation(),
        defer_write=True,
    )

    assert result is not None
    assert not result.get("tacticalAbort", False)
    assert not result.get("attackSuppressed", False)
    assert "attack" in result
    attack_payload = next(
        payload
        for path, payload in result["_deferredWriteEntries"]
        if path.parent.name == "FlightPath" and int(payload.get("pathID")) == 201
    )
    assert any(
        int((waypoint.get("attack") or {}).get("targetID") or 0) == 88
        for waypoint in attack_payload["lahWaypointList"]
    )
    assert imp_data["individualMissionList"][0]["individualMissionInfo"][
        "individualMissionType"
    ] == 2


def test_attacker_sequence_no_route_never_drops_the_attack(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """The production sequence builder must degrade to direct attack."""

    live = {"latitude": 37.2, "longitude": 127.2, "altitude": 420}
    target = {"latitude": 37.5, "longitude": 127.5, "altitude": 700}
    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: {
            "applied": False,
            "status": "no_route",
            "failureCodes": ["NO_HIDE_THEN_RECONNECT_ROUTE"],
        },
    )
    monkeypatch.setattr(
        pipeline,
        "_split_done_resume_lah_path",
        lambda *_args, **_kwargs: ([], [], None),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: (
            [],
            [],
            {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0},
        ),
    )
    monkeypatch.setattr(
        pipeline,
        "_trim_lah_follow_up_paths_after_anchor",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        pipeline,
        "_validate_generated_artifact_write_entries",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )

    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "coordinateList": [],
        },
    }
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [pipeline._default_lah_waypoint_template()],
    }

    result = pipeline._build_lah_attack_sequence_package(
        descriptor={
            "mode": "LAH_ATTACK",
            "label": "manned",
            "aircraft_id": 2,
            "target_id": 88,
            "target_type": 1,
        },
        assigned_targets=[
            {
                "target_id": 88,
                "target_type": 1,
                "coordinate": dict(target),
                "attack_coord": dict(target),
                "selected_weapon_type": 2,
            }
        ],
        new_imp_id=501,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        ctx={"mission_ids": [77]},
        state={"coordinate": dict(live), "speed": 40.0, "heading": 0.0},
        aircraft_id=2,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_Reservation(),
        defer_write=True,
    )

    assert result is not None
    assert [item["targetID"] for item in result["attackSequence"]] == [88]
    assert not result.get("deferredAttackTargetIDs")
    assert imp_data["individualMissionList"][0]["individualMissionInfo"][
        "individualMissionType"
    ] == 2


def test_attacker_builder_prepends_hide_before_existing_attack(
    monkeypatch,
    tmp_path: Path,
) -> None:
    captured: dict[str, dict] = {}
    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: _certified_plan(),
    )
    monkeypatch.setattr(
        pipeline,
        "_split_done_resume_lah_path",
        lambda *_args, **_kwargs: ([], [], None),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: ([], [], {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0}),
    )
    monkeypatch.setattr(pipeline, "_trim_lah_follow_up_paths_after_anchor", lambda **_kwargs: None)
    monkeypatch.setattr(
        pipeline,
        "_trim_lah_resume_waypoints_after_attack_anchor",
        lambda waypoints, **_kwargs: (waypoints, None),
    )
    monkeypatch.setattr(pipeline, "_lah_special_battle_anchor_for_input", lambda *_args, **_kwargs: None)

    def fake_mission_route(*, start_coord, attack_coord, source_plan_id):
        captured["start"] = dict(start_coord)
        return [dict(start_coord), dict(attack_coord)], {
            "constrained": True,
            "startInside": True,
            "attackInside": True,
            "reason": "test",
        }

    monkeypatch.setattr(pipeline, "_build_lah_mission_constrained_attack_route", fake_mission_route)

    def fake_attack_waypoints(
        *,
        template_wp,
        start_coord,
        attack_coord,
        attack_waypoint_id,
        waypoint_id_provider,
        target_id,
        weapon_type,
        **_kwargs,
    ):
        start = deepcopy(template_wp)
        start.update(
            {
                "waypointID": waypoint_id_provider(),
                "coordinate": dict(start_coord),
                "eta": 0,
                "attack": {"targetID": 0, "weaponType": 0},
            }
        )
        attack = deepcopy(template_wp)
        attack.update(
            {
                "waypointID": attack_waypoint_id,
                "coordinate": dict(attack_coord),
                "eta": 20,
                "attack": {"targetID": int(target_id), "weaponType": int(weapon_type)},
            }
        )
        return [start, attack]

    monkeypatch.setattr(pipeline, "_build_lah_low_level_attack_waypoints", fake_attack_waypoints)
    monkeypatch.setattr(pipeline, "_validate_generated_artifact_write_entries", lambda **_kwargs: None)
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    monkeypatch.setattr(pipeline, "get_runtime_attack_int", lambda _key, default: default)
    monkeypatch.setattr(pipeline, "get_runtime_attack_float", lambda _key, default: default)
    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "coordinateList": [],
        },
    }
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [pipeline._default_lah_waypoint_template()],
    }

    result = pipeline._build_lah_attack_package(
        descriptor={
            "mode": "LAH_ATTACK",
            "label": "manned",
            "aircraft_id": 2,
            "target_id": 88,
            "target_type": 1,
        },
        new_imp_id=501,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        attack_coord={"latitude": 37.01, "longitude": 127.01, "altitude": 700},
        ctx={"mission_ids": [77], "_selected_attack_weapon_type": 2},
        state={
            "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800},
            "speed": 40.0,
            "heading": 0.0,
        },
        aircraft_id=2,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_Reservation(),
        defer_write=True,
    )

    assert result is not None and "hidePrelude" in result
    assert captured["start"]["latitude"] == 37.001
    assert captured["start"]["longitude"] == 127.001
    mission_info = imp_data["individualMissionList"][0]["individualMissionInfo"]
    assert mission_info["individualMissionType"] == 2
    assert mission_info["patternType"] == 2
    attack_payload = next(
        payload
        for path, payload in result["_deferredWriteEntries"]
        if path.parent.name == "FlightPath" and int(payload.get("pathID")) == 201
    )
    waypoints = attack_payload["lahWaypointList"]
    assert [item["attack"]["targetID"] for item in waypoints[:-1]] == [0] * (len(waypoints) - 1)
    assert waypoints[-1]["attack"] == {"targetID": 88, "weaponType": 2}
    assert all(right["eta"] >= left["eta"] for left, right in zip(waypoints, waypoints[1:]))


def test_runtime_multi_target_sequence_is_cover_popup_cover_and_ignores_remote_anchor(
    monkeypatch,
    tmp_path: Path,
) -> None:
    class _SequenceReservation:
        def __init__(self) -> None:
            self.path_id = 200
            self.individual_id = 300
            self.waypoint_id = 1000

        def next_paths(self, _aircraft_id: int, count: int):
            values = tuple(range(self.path_id + 1, self.path_id + count + 1))
            self.path_id += count
            return values

        def next_individuals(self, count: int):
            values = tuple(range(self.individual_id + 1, self.individual_id + count + 1))
            self.individual_id += count
            return values

        def next_waypoint(self) -> int:
            self.waypoint_id += 1
            return self.waypoint_id

        def next_path(self, _aircraft_id: int) -> int:
            return self.next_paths(_aircraft_id, 1)[0]

        def next_individual(self) -> int:
            return self.next_individuals(1)[0]

    plan = _certified_plan()
    hide = dict(plan["endpoint"])
    enemy = {"latitude": 37.02, "longitude": 127.02, "altitude": 600}
    remote_anchor = {"latitude": 38.0, "longitude": 129.0, "altitude": 900}
    monkeypatch.setattr(
        pipeline,
        "_plan_lah_enemy_contact_response",
        lambda *_args, **_kwargs: deepcopy(plan),
    )
    monkeypatch.setattr(
        pipeline,
        "_attack_coordinate_at_hide_endpoint",
        lambda *_args, **_kwargs: {
            "latitude": hide["latitude"],
            "longitude": hide["longitude"],
            "altitude": 700,
            "attack_point_at_hide_endpoint": True,
            "attack_point_vertical_popup": True,
        },
    )
    monkeypatch.setattr(
        pipeline,
        "_lah_special_battle_anchor_for_input",
        lambda *_args, **_kwargs: dict(remote_anchor),
    )
    monkeypatch.setattr(
        pipeline,
        "_split_done_resume_lah_path",
        lambda *_args, **_kwargs: ([], [], None),
    )
    monkeypatch.setattr(
        pipeline,
        "_collect_attack_follow_up_replan_artifacts",
        lambda **_kwargs: ([], [], {"preservedCount": 0, "clonedCount": 0, "skippedCount": 0}),
    )
    monkeypatch.setattr(pipeline, "_trim_lah_follow_up_paths_after_anchor", lambda **_kwargs: None)
    monkeypatch.setattr(pipeline, "_record_lah_tactical_points", lambda **_kwargs: None)
    monkeypatch.setattr(pipeline, "_validate_generated_artifact_write_entries", lambda **_kwargs: None)
    monkeypatch.setattr(
        pipeline.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    target_mission = {
        "individualMissionID": 10,
        "pathID": 20,
        "isDone": False,
        "relatedMission": {"inputMissionID": 77, "priorMissionID": 0},
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "coordinateList": [],
        },
    }
    imp_data = {"individualMissionList": [target_mission]}
    fp_data = {
        "pathID": 20,
        "Source": "DSS",
        "lahWaypointList": [pipeline._default_lah_waypoint_template()],
    }

    result = pipeline._build_lah_attack_sequence_package(
        descriptor={
            "mode": "LAH_ATTACK",
            "label": "manned",
            "aircraft_id": 2,
            "target_id": 88,
            "target_type": 1,
        },
        assigned_targets=[
            {
                "target_id": 88,
                "target_type": 1,
                "coordinate": dict(enemy),
                "attack_coord": {**enemy, "altitude": 1200},
                "selected_weapon_type": 2,
            },
            {
                "target_id": 89,
                "target_type": 1,
                "coordinate": {**enemy, "latitude": enemy["latitude"] + 0.001},
                "attack_coord": {**enemy, "latitude": enemy["latitude"] + 0.001, "altitude": 1200},
                "selected_weapon_type": 2,
            },
        ],
        new_imp_id=501,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=0,
        ctx={"mission_ids": [77]},
        state={
            "coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800},
            "speed": 40.0,
            "heading": 0.0,
        },
        aircraft_id=2,
        artifacts=SimpleNamespace(source_plan_id=70),
        emit=lambda _message: None,
        now_ms=1234,
        done_input_ids=set(),
        id_reservation=_SequenceReservation(),
        defer_write=True,
    )

    assert result is not None
    assert [item["targetID"] for item in result["attackSequence"]] == [88, 89]
    assert len({item["pathID"] for item in result["attackSequence"]}) == 2
    for sequence in result["attackSequence"]:
        assert sequence["missionZoneRoute"]["reason"] == "tactical_vertical_popup"
        assert sequence["attackCoordinate"]["latitude"] == pytest.approx(hide["latitude"])
        assert sequence["attackCoordinate"]["longitude"] == pytest.approx(hide["longitude"])
        attack_path_id = int(sequence["pathID"])
        attack_payload = next(
            payload
            for path, payload in result["_deferredWriteEntries"]
            if path.parent.name == "FlightPath" and int(payload.get("pathID")) == attack_path_id
        )
        waypoints = attack_payload["lahWaypointList"]
        attack_index = next(
            index
            for index, waypoint in enumerate(waypoints)
            if int(waypoint["attack"]["targetID"]) == int(sequence["targetID"])
        )
        before, attack, after = waypoints[attack_index - 1 : attack_index + 2]
        assert before["hovering"]["time"] > 0
        assert after["hovering"]["time"] > 0
        assert before["coordinate"]["latitude"] == pytest.approx(hide["latitude"])
        assert attack["coordinate"]["latitude"] == pytest.approx(hide["latitude"])
        assert after["coordinate"]["latitude"] == pytest.approx(hide["latitude"])
        assert int(before["coordinate"]["altitude"]) < int(attack["coordinate"]["altitude"])
        assert int(after["coordinate"]["altitude"]) == int(before["coordinate"]["altitude"])
    assert all(
        waypoint["coordinate"]["latitude"] != pytest.approx(remote_anchor["latitude"])
        for path, payload in result["_deferredWriteEntries"]
        if path.parent.name == "FlightPath"
        for waypoint in payload.get("lahWaypointList", [])
    )


def test_one_replan_defers_third_target_when_only_two_rounds_exist() -> None:
    targets = [
        {
            "target_id": target_id,
            "target_type": 1,
            "coordinate": {
                "latitude": 37.80 + target_id * 0.001,
                "longitude": 128.10,
                "altitude": 500,
            },
        }
        for target_id in (1, 2, 3)
    ]
    aircraft = [
        {
            "aircraft_id": aircraft_id,
            "coordinate": {
                "latitude": 37.79,
                "longitude": 128.10 + aircraft_id * 0.001,
                "altitude": 700,
            },
            # One round each catches the boundary where a third contact must be
            # deferred without invalidating the two assignments already made.
            "weapon_inventory": {"type1": 1, "type2": 0, "type3": 0},
        }
        for aircraft_id in (2, 3)
    ]

    sequences, error = pipeline._assign_targets_to_manned_sequences(targets, aircraft)

    assert error is None
    assert sorted(sequences) == [2, 3]
    assert all(len(sequence) == 1 for sequence in sequences.values())
    assert sum(len(sequence) for sequence in sequences.values()) == 2


def test_one_replan_assigns_three_targets_as_two_plus_one_strikes() -> None:
    targets = [
        {
            "target_id": target_id,
            "target_type": 1,
            "coordinate": {
                "latitude": 37.80 + target_id * 0.001,
                "longitude": 128.10,
                "altitude": 500,
            },
        }
        for target_id in (1, 2, 3)
    ]
    aircraft = [
        {
            "aircraft_id": aircraft_id,
            "coordinate": {
                "latitude": 37.79,
                "longitude": 128.10 + aircraft_id * 0.001,
                "altitude": 700,
            },
            "weapon_inventory": {"type1": 2, "type2": 0, "type3": 0},
        }
        for aircraft_id in (2, 3)
    ]

    sequences, error = pipeline._assign_targets_to_manned_sequences(targets, aircraft)

    assert error is None
    assert sorted(sequences) == [2, 3]
    assert sorted(len(sequence) for sequence in sequences.values()) == [1, 2]
    assert sum(len(sequence) for sequence in sequences.values()) == 3
    assert sorted(
        int(target["target_id"])
        for sequence in sequences.values()
        for target in sequence
    ) == [1, 2, 3]


def test_attack_continuity_expected_pairs_follow_each_lah_sequence() -> None:
    sequences = {
        2: [{"target_id": 8}],
        3: [{"target_id": 7}],
    }

    assert pipeline._expected_attack_pairs_from_manned_sequences(sequences) == {
        (2, 8),
        (3, 7),
    }


def test_attack_continuity_expected_pairs_keep_two_strikes_on_one_lah() -> None:
    sequences = {
        2: [{"target_id": 8}, {"targetID": 9}],
        3: [{"target_id": 7}],
    }

    assert pipeline._expected_attack_pairs_from_manned_sequences(sequences) == {
        (2, 8),
        (2, 9),
        (3, 7),
    }


def test_missing_speed_fails_before_dem_lookup(monkeypatch) -> None:
    monkeypatch.setattr(
        lah_enemy_contact,
        "regional_dem_path_for_coordinate",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("DEM must not open")),
    )

    result = lah_enemy_contact.plan_enemy_contact_response(
        2,
        {"coordinate": {"latitude": 37.0, "longitude": 127.0, "altitude": 800}},
        [],
        [],
        role="attacker",
    )

    assert result["applied"] is False
    assert result["failureCodes"] == ["MISSING_OWN_SPEED"]
    assert result["routeWaypoints"] == []


class _FlatNativeDem:
    def __init__(self) -> None:
        self.block_elev = np.zeros((400, 400), dtype=np.float32)
        self.height = self.width = 400
        self.transform = Affine(10.0, 0.0, 0.0, 0.0, -10.0, 4000.0)
        self.inverse_transform = ~self.transform
        self.cell_m = self.cell_x_m = self.cell_y_m = 10.0

    def contains_native(self, x: float, y: float) -> bool:
        col, row = self.inverse_transform * (float(x), float(y))
        return 0.0 <= row < self.height and 0.0 <= col < self.width

    def nearest_index(self, x: float, y: float) -> tuple[int, int]:
        col, row = self.inverse_transform * (float(x), float(y))
        return (
            int(np.clip(round(row - 0.5), 0, self.height - 1)),
            int(np.clip(round(col - 0.5), 0, self.width - 1)),
        )

    def native_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return float(y) / 111_132.92, float(x) / 88_000.0

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        x, y = self.transform * (float(col) + 0.5, float(row) + 0.5)
        return float(x), float(y)


def test_moving_aircraft_can_turn_to_endpoint_behind_it() -> None:
    dem = _FlatNativeDem()
    own_x, own_y = dem.cell_center(300, 100)
    goal_x, goal_y = dem.cell_center(200, 100)
    endpoint = HideEndpointCandidate(
        x=goal_x,
        y=goal_y,
        row=200.0,
        col=100.0,
        ground_m=0.0,
        min_altitude_m=50.0,
        max_altitude_m=500.0,
        preferred_altitude_m=100.0,
        horizontal_distance_m=1000.0,
    )

    result = plan_hide_communication_routes(
        dem,
        own_x,
        own_y,
        100.0,
        37.0,
        180.0,
        [endpoint],
        deadline_s=60.0,
        dynamics=RouteDynamics(max_endpoint_candidates=1),
    )

    assert result.green_valid
    assert result.selected is not None
    assert result.selected.kind.startswith("arc_")
    assert result.selected.terrain_safe
    assert result.selected.dynamics_feasible


def test_route_takes_late_cover_but_never_accepts_re_exposure(monkeypatch) -> None:
    class ScheduleDem:
        height = width = 100
        cell_m = 10.0

        @staticmethod
        def native_to_rowcol(x: float, y: float):
            return float(y) / 10.0 + 0.5, float(x) / 10.0 + 0.5

    class ScheduleConfig:
        los_samples_per_cell = 1.2
        los_max_steps = 128
        max_analysis_enemies = 3
        weapon_range_m = 5000.0

        def with_overrides(self, **_kwargs):
            return self

    route = SimpleNamespace(
        waypoints=[
            SimpleNamespace(x=0.0, y=0.0, alt_m=100.0, timestamp_s=0.0),
            SimpleNamespace(x=100.0, y=0.0, alt_m=100.0, timestamp_s=20.0),
        ]
    )

    monkeypatch.setattr(
        hide_com_refine,
        "_observer_requirements_chunked",
        lambda _dem, _config, _enemy, **kwargs: np.where(
            kwargs["xs"] >= 25.0,
            200.0,
            50.0,
        ),
    )
    valid = lah_enemy_contact._route_concealment_schedule(
        ScheduleDem(),
        ScheduleConfig(),
        [object()],
        route,
        hide_deadline_s=10.0,
        planning_elapsed_s=1.0,
    )
    assert valid["valid"] is True
    assert valid["hideAchievedS"] <= 10.0
    assert valid["continuousAfterHide"] is True

    monkeypatch.setattr(
        hide_com_refine,
        "_observer_requirements_chunked",
        lambda _dem, _config, _enemy, **kwargs: np.where(
            (kwargs["xs"] >= 25.0) & (kwargs["xs"] < 75.0),
            200.0,
            50.0,
        ),
    )
    reexposed = lah_enemy_contact._route_concealment_schedule(
        ScheduleDem(),
        ScheduleConfig(),
        [object()],
        route,
        hide_deadline_s=10.0,
        planning_elapsed_s=1.0,
    )
    # Re-exposure means the route never reaches *sustained* cover, which is a
    # real failure - unlike merely being slow.
    assert reexposed["valid"] is False
    assert reexposed["failureCode"] == "ROUTE_NEVER_REACHES_CONCEALMENT"


def test_endpoint_selection_ceiling_stays_below_route_verification_ceiling() -> None:
    """An endpoint accepted by the refinement must survive the route re-check.

    Both stages recompute the enemy threshold, but each derives its LOS step
    count from its own sample batch, so the same point differs by a fraction of
    a metre between them.  The refinement clips the preferred altitude straight
    to its ceiling, so unless that ceiling sits below the verification ceiling
    every candidate is rejected and concealment is never applied at all.
    """

    selection_margin_m = (
        hide_com_refine._ALTITUDE_MARGIN_M
        + hide_com_refine._ENEMY_CEILING_SELECTION_MARGIN_M
    )
    verification_margin_m = lah_enemy_contact._ROUTE_CONCEALMENT_VERIFY_MARGIN_M
    assert selection_margin_m > verification_margin_m
    # Observed cross-stage spread on the operational DEM is ~0.5 m.
    assert selection_margin_m - verification_margin_m >= 1.0


def test_operational_geometry_yields_a_verified_concealment_route() -> None:
    """End-to-end: a certified endpoint must also be hidden along the route.

    Uses the geometry from the 2026-07-25 field run, where every candidate was
    silently rejected because the accepted endpoint sat exactly on the
    concealment boundary.
    """

    own_state = {
        "coordinate": {
            "latitude": 37.95906958200593,
            "longitude": 127.31697513002078,
            "altitude": 367.0,
        },
        "speed": 0.0,
        "heading": 26.0,
    }
    dem_path = regional_dem_path_for_coordinate(
        Path(__file__).resolve().parents[3] / "resource",
        own_state["coordinate"]["latitude"],
        own_state["coordinate"]["longitude"],
    )
    if dem_path is None or not Path(dem_path).is_file():
        pytest.skip("operational regional DEM is not installed")

    uav_states = [
        {"aircraft_id": 4, "coordinate": {"latitude": 37.96838842629084, "longitude": 127.3249679183796, "altitude": 1167.0}},
        {"aircraft_id": 5, "coordinate": {"latitude": 37.97499924791551, "longitude": 127.31957377191267, "altitude": 1173.0}},
        {"aircraft_id": 6, "coordinate": {"latitude": 37.96540752206648, "longitude": 127.32595460522016, "altitude": 1188.0}},
    ]
    enemy_coordinates = [
        {"coordinate": {"latitude": 37.97138154195951, "longitude": 127.32839121305919, "altitude": 0.0}},
        {"coordinate": {"latitude": 37.972898607055214, "longitude": 127.32662291056117, "altitude": 0.0}},
        {"coordinate": {"latitude": 37.971881100108746, "longitude": 127.3308616937868, "altitude": 0.0}},
    ]

    # The 10 s default is unreachable from a hold 200 m above the concealment
    # ceiling; this pins the concealment gate, not the response deadline.
    result = lah_enemy_contact.plan_enemy_contact_response(
        1,
        own_state,
        uav_states,
        enemy_coordinates,
        role="relay",
        min_uav_links=3,
        deadline_s=60.0,
    )

    assert result["applied"] is True, result.get("detail")
    assert result["enemyVisibleCount"] == 0
    assert result["uavLinkCount"] >= 3
    assert result["guarantees"]["continuousEnemyMaskingAfterHide"] is True
    assert result["concealmentValidation"]["hiddenSampleCount"] >= 1
