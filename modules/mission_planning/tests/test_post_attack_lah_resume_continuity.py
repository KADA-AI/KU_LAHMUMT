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


def test_closed_target_selection_does_not_require_tracking_input_match() -> None:
    attack = _mission(1, 101, input_id=3, target_id=11)

    selected = post_attack._lah_attack_target_mission_indices(
        [attack],
        current_input_id=4,
        target_id=11,
    )

    assert selected == [0]


def test_post_attack_connector_uses_the_planned_popup_base_not_live_peak(
    monkeypatch,
) -> None:
    armed = _mission(1, 101, input_id=3, target_id=11)
    armed["individualMissionInfo"]["individualMissionType"] = 2
    base = _waypoint(
        1001,
        38.01145277810638,
        127.30210272704713,
        419,
        attack_target=11,
    )
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda path_id, **_kwargs: (
            {"pathID": 101, "lahWaypointList": [base]}
            if int(path_id) == 101
            else None
        ),
    )

    live_peak = {
        "latitude": 38.0114528,
        "longitude": 127.3021027,
        "altitude": 2327,
    }
    rebased, detail = post_attack._planned_attack_popup_base_for_post_attack(
        current_coord=live_peak,
        mission_list=[armed],
        attack_mission_indices=[0],
        run_cache=None,
    )

    assert detail is not None
    assert detail["liveAltitudeM"] == 2327
    assert detail["baseAltitudeM"] == 419
    assert detail["targetID"] == 11
    assert rebased is not None
    assert rebased["latitude"] == live_peak["latitude"]
    assert rebased["longitude"] == live_peak["longitude"]
    assert rebased["altitude"] == 419


def test_post_attack_does_not_rebase_after_leaving_the_attack_point(
    monkeypatch,
) -> None:
    armed = _mission(1, 101, input_id=3, target_id=11)
    base = _waypoint(1001, 38.0, 127.0, 419, attack_target=11)
    monkeypatch.setattr(
        post_attack,
        "_load_path_payload",
        lambda _path_id, **_kwargs: {
            "pathID": 101,
            "lahWaypointList": [base],
        },
    )
    departed = {"latitude": 38.01, "longitude": 127.01, "altitude": 900}

    unchanged, detail = post_attack._planned_attack_popup_base_for_post_attack(
        current_coord=departed,
        mission_list=[armed],
        attack_mission_indices=[0],
        run_cache=None,
    )

    assert detail is None
    assert unchanged == departed


def test_returning_lahs_are_found_by_closed_target_not_tracking_input(
    monkeypatch,
) -> None:
    packages = {
        1: [_mission(1, 101, input_id=3, target_id=11)],
        2: [
            _mission(2, 201, input_id=3, target_id=9),
            _mission(3, 202, input_id=3, target_id=11),
        ],
        3: [_mission(4, 301, input_id=3, target_id=11)],
    }
    monkeypatch.setattr(
        post_attack,
        "_known_destroyed_target_ids",
        lambda _target_id=None: {9, 11},
    )
    monkeypatch.setattr(
        post_attack,
        "_load_imp_package_for_aircraft_cached",
        lambda *, aircraft_id, **_kwargs: {
            "individualMissionList": packages[int(aircraft_id)]
        },
    )

    returning = post_attack._find_returning_manned_attack_aircraft_ids(
        current_plan_id=700000016,
        current_input_id=4,
        target_id=11,
        plan_data={
            "aircraftList": [
                {"aircraftID": 1},
                {"aircraftID": 2},
                {"aircraftID": 3},
                {"aircraftID": 4},
            ]
        },
    )

    assert returning == [1, 2, 3]


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


def test_closing_queued_attack_prunes_it_and_preserves_remaining_identity(
    monkeypatch,
    tmp_path: Path,
) -> None:
    remaining = _mission(14, 140, target_id=14)
    remaining["individualMissionInfo"]["individualMissionType"] = 2
    older_destroyed = _mission(12, 120, input_id=3, target_id=12)
    older_destroyed["individualMissionInfo"]["individualMissionType"] = 2
    closed = _mission(10, 100, input_id=3, target_id=10)
    closed["individualMissionInfo"]["individualMissionType"] = 2
    resume = _mission(15, 150, target_id=0, post_attack_resume=True)
    resume["individualMissionInfo"]["targetID"] = None
    paths = {
        100: {"pathID": 100, "lahWaypointList": [_waypoint(1001, 38.0, 127.0, 600, attack_target=10)]},
        120: {"pathID": 120, "lahWaypointList": [_waypoint(1201, 38.0, 127.0, 605, attack_target=12)]},
        140: {"pathID": 140, "lahWaypointList": [_waypoint(1401, 38.0, 127.0, 610, attack_target=14)]},
        150: {"pathID": 150, "lahWaypointList": [_waypoint(1501, 38.0, 127.0, 550)]},
    }
    source_imp = {
        "individualMissionPackageID": 800000001,
        "aircraftID": 3,
        "individualMissionList": [remaining, older_destroyed, closed, resume],
    }
    monkeypatch.setattr(
        post_attack,
        "_load_imp_package_for_aircraft_cached",
        lambda **_kwargs: source_imp,
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
            individual_mission_id=14,
            path_id=140,
        ),
    )
    monkeypatch.setattr(
        post_attack,
        "_known_destroyed_target_ids",
        lambda _target_id=None: {10, 12},
    )

    class _Reservation:
        def next_imp(self) -> int:
            return 800000002

    written: list[tuple[Path, dict]] = []
    monkeypatch.setattr(
        post_attack.ReplanIdReservation,
        "reserve",
        lambda **_kwargs: _Reservation(),
    )
    monkeypatch.setattr(
        post_attack,
        "_validate_generated_post_attack_artifact_payloads",
        lambda **_kwargs: {},
    )
    monkeypatch.setattr(
        post_attack,
        "_write_or_defer_post_attack_json_batch",
        lambda entries, **_kwargs: written.extend(entries),
    )
    monkeypatch.setattr(
        post_attack.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )

    result = post_attack._build_post_attack_lah_resume_update(
        source_plan_id=700000001,
        current_input_id=4,
        target_id=10,
        aircraft_id=3,
        current_state={"current_waypoint_id": 1401},
        now_ms=1,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert result is not None
    assert result["aircraft_id"] == 3
    assert result["sourceIndividualMissionPackageID"] == 800000001
    assert result["individualMissionPackageID"] == 800000002
    assert result["continuityRepackaged"] is True
    assert result["remainingAttackTargetIDs"] == [14]
    assert result["generatedPathIDs"] == []

    assert len(written) == 1
    replacement_imp = written[0][1]
    replacement_missions = replacement_imp["individualMissionList"]
    assert [
        mission["individualMissionID"] for mission in replacement_missions
    ] == [14, 15]
    # The surviving attack keeps the exact committed mission/path identity.
    assert replacement_missions[0]["pathID"] == 140
    assert replacement_missions[0]["individualMissionInfo"]["targetID"] == 14
    assert all(
        mission.get("individualMissionInfo", {}).get("targetID") not in {10, 12}
        for mission in replacement_missions
    )


def test_closing_current_attack_keeps_only_a_nonfiring_safety_suffix(
    monkeypatch,
    tmp_path: Path,
) -> None:
    closed = _mission(10, 100, target_id=10)
    closed["individualMissionInfo"]["individualMissionType"] = 2
    remaining = _mission(14, 140, target_id=14)
    remaining["individualMissionInfo"]["individualMissionType"] = 2
    resume = _mission(15, 150, target_id=0, post_attack_resume=True)
    resume["individualMissionInfo"]["targetID"] = None
    attack_wp = _waypoint(1001, 38.0, 127.0, 600, attack_target=10)
    descent_wp = _waypoint(1002, 38.001, 127.0, 500)
    paths = {
        100: {
            "pathID": 100,
            "aircraftID": 3,
            "individualMissionID": 10,
            "lahWaypointList": [attack_wp, descent_wp],
        },
        140: {
            "pathID": 140,
            "lahWaypointList": [
                _waypoint(1401, 38.0, 127.0, 610, attack_target=14)
            ],
        },
        150: {
            "pathID": 150,
            "lahWaypointList": [_waypoint(1501, 38.0, 127.0, 550)],
        },
    }
    monkeypatch.setattr(
        post_attack,
        "_load_imp_package_for_aircraft_cached",
        lambda **_kwargs: {
            "individualMissionPackageID": 800000001,
            "aircraftID": 3,
            "individualMissionList": [closed, remaining, resume],
        },
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
            current_waypoint_id=1001,
            previous_waypoint_id=None,
        ),
    )
    monkeypatch.setattr(
        post_attack,
        "_known_destroyed_target_ids",
        lambda _target_id=None: {10},
    )

    class _Reservation:
        def __init__(self) -> None:
            self._waypoint = 2000

        def next_imp(self) -> int:
            return 800000002

        def next_individual(self) -> int:
            return 99

        def next_path(self, _aircraft_id: int) -> int:
            return 999

        def next_waypoint(self) -> int:
            self._waypoint += 1
            return self._waypoint

    written: list[tuple[Path, dict]] = []
    monkeypatch.setattr(
        post_attack.ReplanIdReservation,
        "reserve",
        lambda **_kwargs: _Reservation(),
    )
    monkeypatch.setattr(
        post_attack,
        "_validate_generated_post_attack_artifact_payloads",
        lambda **_kwargs: {},
    )
    monkeypatch.setattr(
        post_attack,
        "_write_or_defer_post_attack_json_batch",
        lambda entries, **_kwargs: written.extend(entries),
    )
    monkeypatch.setattr(
        post_attack.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )

    result = post_attack._build_post_attack_lah_resume_update(
        source_plan_id=700000001,
        current_input_id=4,
        target_id=10,
        aircraft_id=3,
        current_state={
            "current_waypoint_id": 1001,
            "coordinate": attack_wp["coordinate"],
        },
        now_ms=1,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert result is not None
    assert result["remainingAttackTargetIDs"] == [14]
    assert result["generatedPathIDs"] == [999]
    replacement_imp = next(
        payload
        for path, payload in written
        if path.parent.name == "IndividualMissionPlan"
    )
    replacement_missions = replacement_imp["individualMissionList"]
    assert [
        mission["individualMissionID"] for mission in replacement_missions
    ] == [99, 14, 15]
    assert replacement_missions[0]["postAttackResume"] is True
    assert replacement_missions[0]["individualMissionInfo"]["targetID"] is None
    assert replacement_missions[1]["individualMissionID"] == 14
    assert replacement_missions[1]["pathID"] == 140

    safety_path = next(
        payload for path, payload in written if path.parent.name == "FlightPath"
    )
    safety_waypoints = safety_path["lahWaypointList"]
    assert len(safety_waypoints) == 1
    assert safety_waypoints[0]["waypointID"] == 2001
    assert safety_waypoints[0]["coordinate"] == descent_wp["coordinate"]
    assert safety_waypoints[0]["attack"] == {"targetID": 0, "weaponType": 0}


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
