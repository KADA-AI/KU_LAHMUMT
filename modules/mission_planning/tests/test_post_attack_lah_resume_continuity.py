from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from modules.mission_planning.replanning.triggers.attack import pipeline as attack
from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack


def _waypoint(
    waypoint_id: int,
    latitude: float,
    longitude: float,
    altitude: int,
    *,
    attack_target: int = 0,
) -> dict:
    return {
        "waypointID": int(waypoint_id),
        "nextWaypointID": 0,
        "isDone": False,
        "coordinate": {
            "latitude": float(latitude),
            "longitude": float(longitude),
            "altitude": int(altitude),
        },
        "speed": 40.0,
        "eta": 0,
        "ecf": 0.0,
        "hovering": {"time": 0},
        "attack": {
            "targetID": int(attack_target),
            "weaponType": 1 if attack_target else 0,
        },
    }


def _mission(
    mission_id: int,
    path_id: int,
    *,
    input_id: int = 4,
    target_id: int = 14,
    post_attack_resume: bool = False,
) -> dict:
    mission = {
        "individualMissionID": int(mission_id),
        "pathID": int(path_id),
        "isDone": False,
        "relatedMission": {"inputMissionID": int(input_id)},
        "individualMissionInfo": {
            "individualMissionType": 9,
            "patternType": 12,
            "targetID": int(target_id),
            "coordinateList": [],
        },
    }
    if post_attack_resume:
        mission["postAttackResume"] = True
    return mission


def test_explicit_attack_resume_is_not_removed_with_its_target_branch() -> None:
    missions = [
        _mission(1, 101),
        _mission(2, 102, post_attack_resume=True),
    ]

    selected = post_attack._lah_attack_target_mission_indices(
        missions,
        current_input_id=4,
        target_id=14,
    )

    assert selected == [0]
    assert missions[1]["postAttackResume"] is True


def test_legacy_target_bound_resume_is_preserved_by_path_semantics(monkeypatch) -> None:
    missions = [_mission(1, 101), _mission(2, 102)]
    paths = {
        101: {"lahWaypointList": [_waypoint(11, 38.0, 127.0, 600, attack_target=14)]},
        102: {"lahWaypointList": [_waypoint(12, 38.01, 127.01, 550)]},
    }
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda path_id, **_kwargs: paths.get(int(path_id)),
    )

    selected = post_attack._preserve_legacy_lah_target_bound_resumes(
        missions,
        [0, 1],
    )

    assert selected == [0]
    assert missions[1]["postAttackResume"] is True
    assert missions[1]["postAttackSourceTargetID"] == 14
    assert missions[1]["individualMissionInfo"]["targetID"] is None


def test_a_single_support_hold_is_not_mistaken_for_a_resume(monkeypatch) -> None:
    missions = [_mission(1, 101)]
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda _path_id, **_kwargs: {
            "lahWaypointList": [_waypoint(11, 38.0, 127.0, 600)]
        },
    )

    selected = post_attack._preserve_legacy_lah_target_bound_resumes(
        missions,
        [0],
    )

    assert selected == [0]


def test_a_marked_resume_is_removed_when_a_new_attack_rebuilds_the_same_input() -> None:
    resume = _mission(2, 102, post_attack_resume=True)
    resume["individualMissionInfo"]["targetID"] = None
    future = _mission(3, 103, input_id=5, target_id=0, post_attack_resume=True)

    kept = attack._drop_stale_lah_tactical_follow_ups(
        [resume, future],
        current_input_id=4,
    )

    assert kept == [future]


def test_global_attack_target_scan_ignores_a_marked_resume(monkeypatch) -> None:
    action = _mission(1, 101)
    resume = _mission(2, 102, post_attack_resume=True)
    monkeypatch.setattr(
        attack,
        "_load_attack_exclusion_plan_context",
        lambda _plan_id, _aircraft_id: {
            "individualMissionList": [action, resume]
        },
    )

    context = attack._resolve_global_attack_exclusion_lah_context(
        source_plan_id=700000001,
        aircraft_id=2,
    )

    assert context == {"current_input_id": 4, "target_ids": [14]}


def test_closing_first_sequential_attack_preserves_remaining_attack_identity(
    monkeypatch,
) -> None:
    closed = _mission(10, 100, target_id=10)
    closed["individualMissionInfo"]["individualMissionType"] = 2
    remaining = _mission(14, 140, target_id=14)
    remaining["individualMissionInfo"]["individualMissionType"] = 2
    resume = _mission(15, 150, target_id=0, post_attack_resume=True)
    resume["individualMissionInfo"]["targetID"] = None
    paths = {
        100: {"pathID": 100, "lahWaypointList": [_waypoint(1001, 38.0, 127.0, 600, attack_target=10)]},
        140: {"pathID": 140, "lahWaypointList": [_waypoint(1401, 38.0, 127.0, 610, attack_target=14)]},
        150: {"pathID": 150, "lahWaypointList": [_waypoint(1501, 38.0, 127.0, 550)]},
    }
    monkeypatch.setattr(
        post_attack,
        "_load_imp_package_for_aircraft_cached",
        lambda **_kwargs: {"individualMissionList": [closed, remaining, resume]},
    )
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda path_id, **_kwargs: paths.get(int(path_id)),
    )
    monkeypatch.setattr(
        post_attack,
        "_resolve_plan_artifacts",
        lambda **_kwargs: SimpleNamespace(
            individual_mission_id=10,
            path_id=100,
        ),
    )

    result = post_attack._build_post_attack_lah_resume_update(
        source_plan_id=700000001,
        current_input_id=4,
        target_id=10,
        aircraft_id=3,
        current_state={"current_waypoint_id": 1001},
        now_ms=1,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert result == {
        "aircraft_id": 3,
        "preserveExistingPackage": True,
        "continuingAttack": True,
        "remainingAttackTargetIDs": [14],
        "generatedPathIDs": [],
        "reservationSummaries": [],
    }


def test_an_existing_continuous_return_route_needs_no_synthetic_connector(
    monkeypatch,
) -> None:
    boundary = _waypoint(1, 37.989733, 127.303033, 748)
    return_next = _waypoint(2, 37.988000, 127.302700, 700)
    source_mission = _mission(20, 200, post_attack_resume=True)
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda _path_id, **_kwargs: {
            "pathID": 200,
            "lahWaypointList": [boundary, return_next],
        },
    )

    prepared = post_attack._prepare_post_attack_lah_follow_up_connector(
        primary_resume=(_mission(10, 100), {}, [boundary]),
        follow_up_source_missions=[source_mission],
        aircraft_id=2,
        run_cache=None,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert prepared is None


def test_dem_safe_connector_closes_primary_to_follow_up_boundary(monkeypatch) -> None:
    start = _waypoint(1, 38.0000, 127.0000, 700)
    destination = _waypoint(2, 38.0100, 127.0100, 620)
    terminal = _waypoint(3, 38.0200, 127.0200, 610)
    source_mission = _mission(20, 200, input_id=5, target_id=0)
    source_path = {"pathID": 200, "lahWaypointList": [destination, terminal]}
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda _path_id, **_kwargs: source_path,
    )

    midpoint_coord = {
        "latitude": 38.005,
        "longitude": 127.005,
        "altitude": 680,
    }

    def _terrain_route(
        *, template_wp, route_coordinates, waypoint_id_provider, speed_mps, **_kwargs
    ):
        coordinates = [route_coordinates[0], midpoint_coord, route_coordinates[-1]]
        return [
            attack._build_lah_waypoint_from_template(
                template_wp,
                waypoint_id_provider(),
                coordinate,
                0,
                mark_attack=False,
                target_id=None,
                speed_override_mps=speed_mps,
            )
            for coordinate in coordinates
        ]

    monkeypatch.setattr(attack, "_build_lah_low_level_waypoint_route", _terrain_route)

    prepared = post_attack._prepare_post_attack_lah_follow_up_connector(
        primary_resume=(_mission(10, 100), {}, [start]),
        follow_up_source_missions=[source_mission],
        aircraft_id=2,
        run_cache=None,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert prepared is not None
    # The original destination waypoint remains authoritative, so only the
    # prior endpoint and DEM midpoint are inserted before it.
    assert len(prepared["prefixWaypoints"]) == 2

    cloned_mission = _mission(30, 300, input_id=5, target_id=0)
    cloned_path = {
        "pathID": 300,
        "individualMissionID": 30,
        "lahWaypointList": [destination, terminal],
    }
    waypoint_ids = iter(range(1001, 1100))
    inserted = post_attack._prepend_post_attack_lah_follow_up_connector(
        prepared=prepared,
        follow_up_missions=[cloned_mission],
        follow_up_paths=[(Path("300.json"), cloned_path)],
        waypoint_id_provider=lambda: next(waypoint_ids),
        aircraft_id=2,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    combined = cloned_path["lahWaypointList"]
    assert inserted == 2
    assert [row["coordinate"] for row in combined] == [
        start["coordinate"],
        midpoint_coord,
        destination["coordinate"],
        terminal["coordinate"],
    ]
    assert combined[0]["coordinate"] == start["coordinate"]
    assert [row["eta"] for row in combined] == sorted(row["eta"] for row in combined)
    assert len({row["waypointID"] for row in combined}) == len(combined)
    assert [row["nextWaypointID"] for row in combined[:-1]] == [
        row["waypointID"] for row in combined[1:]
    ]
    assert combined[-1]["nextWaypointID"] == 0
    assert cloned_mission["individualMissionInfo"]["coordinateList"] == [
        row["coordinate"] for row in combined
    ]
