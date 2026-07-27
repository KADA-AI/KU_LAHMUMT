"""The run home is flown under cover while contacts are still alive.

The post-attack pipeline used to send every manned aircraft straight back onto
its cruise route the moment one target died, regardless of how many others were
still shooting.  Each aircraft now hides for as long as the remaining strike
takes, and the command aircraft keeps its relay links while it does.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Dict, List

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap
from modules.mission_planning.replanning.triggers.post_attack import pipeline as pa

STATE: Dict[str, Any] = {"coordinate": {"latitude": 37.831, "longitude": 128.114}}


def _entry(target_id: int, lat: float, lon: float, *, destroyed: bool = False, watcher: int = 4):
    return {
        "key": f"{target_id}-{watcher}",
        "target_id": target_id,
        "watcher_id": watcher,
        "coordinate": {"latitude": lat, "longitude": lon, "altitude": 0},
        "is_destroyed": destroyed,
    }


def _use_entries(monkeypatch, entries: List[Dict[str, Any]]) -> None:
    monkeypatch.setattr(ap, "_load_target_entries", lambda: (entries, None))


def test_the_target_just_killed_is_not_treated_as_a_remaining_threat(monkeypatch) -> None:
    _use_entries(monkeypatch, [_entry(11, 37.870, 128.127)])

    assert pa._remaining_enemy_coordinates(11) == []


def test_a_live_contact_is_kept(monkeypatch) -> None:
    _use_entries(
        monkeypatch,
        [_entry(11, 37.870, 128.127), _entry(12, 37.872, 128.129)],
    )

    remaining = pa._remaining_enemy_coordinates(11)

    assert len(remaining) == 1
    assert remaining[0]["coordinate"]["latitude"] == pytest.approx(37.872)


def test_a_kill_reported_by_any_watcher_settles_the_target(monkeypatch) -> None:
    """One observer's stale 'alive' row must not resurrect a dead target."""

    _use_entries(
        monkeypatch,
        [
            _entry(12, 37.872, 128.129, destroyed=True, watcher=4),
            _entry(12, 37.872, 128.129, destroyed=False, watcher=5),
        ],
    )

    assert pa._remaining_enemy_coordinates(11) == []


def test_the_same_target_seen_by_several_watchers_is_one_contact(monkeypatch) -> None:
    _use_entries(
        monkeypatch,
        [
            _entry(12, 37.872, 128.129, watcher=4),
            _entry(12, 37.872, 128.129, watcher=5),
            _entry(12, 37.872, 128.129, watcher=6),
        ],
    )

    assert len(pa._remaining_enemy_coordinates(11)) == 1


def test_an_unreadable_target_file_means_no_cover_rather_than_a_crash(monkeypatch) -> None:
    monkeypatch.setattr(ap, "_load_target_entries", lambda: ([], "missing"))

    assert pa._remaining_enemy_coordinates(11) == []


def test_no_remaining_enemy_leaves_the_return_route_untouched(monkeypatch) -> None:
    _use_entries(monkeypatch, [_entry(11, 37.870, 128.127)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])

    plan, enemies, hold_s = pa._post_attack_cover_prelude(
        aircraft_id=2,
        current_state=STATE,
        destroyed_target_id=11,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert plan is None
    assert enemies == []
    assert hold_s == 0


def test_the_command_aircraft_solves_for_a_relay_and_a_wingman_only_hides(monkeypatch) -> None:
    _use_entries(monkeypatch, [_entry(12, 37.872, 128.129)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])
    roles: List[str] = []
    modes: List[str] = []

    def _capture(descriptor, state, *, role, emit):
        roles.append(str(role))
        modes.append(str(descriptor.get("mode")))
        return None

    monkeypatch.setattr(ap, "_plan_lah_enemy_contact_response", _capture)

    for aircraft_id in (1, 2, 3):
        pa._post_attack_cover_prelude(
            aircraft_id=aircraft_id,
            current_state=STATE,
            destroyed_target_id=11,
            emit=lambda _msg: None,
            log_prefix="[TEST]",
        )

    assert roles == ["relay", "attacker", "attacker"]
    assert modes == ["LAH_RELAY", "LAH_HOLD_RESUME", "LAH_HOLD_RESUME"]


def test_the_planner_receives_bare_coordinates_not_wrapper_rows(monkeypatch) -> None:
    """Wrapped rows normalize to nothing, so every contact would be discarded."""

    from modules.mission_planning.pipelines.lah_enemy_contact import _coordinate

    _use_entries(monkeypatch, [_entry(12, 37.872, 128.129)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])
    captured: List[Dict[str, Any]] = []

    def _capture(descriptor, state, *, role, emit):
        captured.append(descriptor["enemy_contact"])
        return None

    monkeypatch.setattr(ap, "_plan_lah_enemy_contact_response", _capture)

    pa._post_attack_cover_prelude(
        aircraft_id=2,
        current_state=STATE,
        destroyed_target_id=11,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    contact = captured[0]
    assert contact["enemy_coordinate_count"] == 1
    for raw in contact["enemy_coordinates"]:
        assert _coordinate(raw) is not None


def test_the_wait_is_sized_to_the_remaining_strike(monkeypatch) -> None:
    _use_entries(monkeypatch, [_entry(12, 37.872, 128.129)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])
    monkeypatch.setattr(ap, "_plan_lah_enemy_contact_response", lambda *a, **k: None)

    _plan, enemies, hold_s = pa._post_attack_cover_prelude(
        aircraft_id=2,
        current_state=STATE,
        destroyed_target_id=11,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert enemies
    minimum_s = ap.get_runtime_attack_int("lah_wait_hold_min_seconds", 30)
    maximum_s = ap.get_runtime_attack_int("lah_wait_hold_max_seconds", 600)
    assert minimum_s <= hold_s <= maximum_s
    # Closing on the contact plus both cover dwells.
    assert hold_s >= 2 * ap._attack_cover_hold_seconds()


def test_planned_post_shot_descent_is_recognized_as_the_cover_suffix() -> None:
    current = {"latitude": 37.83, "longitude": 128.11, "altitude": 1200}
    attack_wp = {
        "coordinate": dict(current),
        "hovering": {"time": 0},
        "attack": {"targetID": 7, "weaponType": 1},
    }
    cover_wp = {
        "coordinate": {"latitude": 37.83, "longitude": 128.11, "altitude": 650},
        "hovering": {"time": 30},
        "attack": {"targetID": 0, "weaponType": 0},
    }

    waypoint, coordinate = pa._preserved_regain_cover_waypoint(
        [attack_wp, cover_wp], current_coord=current
    )

    assert waypoint is cover_wp
    assert coordinate == cover_wp["coordinate"]


def test_delayed_kill_event_keeps_the_current_cover_waypoint() -> None:
    attack_wp = {
        "waypointID": 101,
        "nextWaypointID": 102,
        "isDone": True,
        "coordinate": {"latitude": 37.83, "longitude": 128.11, "altitude": 1200},
        "hovering": {"time": 0},
        "attack": {"targetID": 7, "weaponType": 1},
    }
    cover_wp = {
        "waypointID": 102,
        "nextWaypointID": 0,
        "isDone": False,
        "coordinate": {"latitude": 37.83, "longitude": 128.11, "altitude": 650},
        "hovering": {"time": 30},
        "attack": {"targetID": 0, "weaponType": 0},
    }
    source = {"aircraftID": 2, "lahWaypointList": [attack_wp, cover_wp]}
    artifacts = SimpleNamespace(current_waypoint_id=102, previous_waypoint_id=101)

    assert pa._current_waypoint_is_planned_regain_cover(
        source["lahWaypointList"], 102
    ) is True
    _done, resume, _removed = ap._split_done_resume_lah_path(
        source,
        artifacts=artifacts,
        current_coord=cover_wp["coordinate"],
        emit=lambda _message: None,
        exclude_current_from_resume=False,
    )
    waypoint, coordinate = pa._preserved_regain_cover_waypoint(
        resume,
        current_coord=cover_wp["coordinate"],
    )

    assert len(resume) == 1
    assert waypoint is not None
    assert coordinate == cover_wp["coordinate"]


def test_preserved_cover_is_revalidated_from_cover_against_current_contacts(
    monkeypatch,
) -> None:
    _use_entries(monkeypatch, [_entry(12, 37.872, 128.129)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])
    captured: Dict[str, Any] = {}

    def _capture(descriptor, state, *, role, emit):
        captured["descriptor"] = descriptor
        captured["state"] = state
        return {
            "applied": True,
            "endpoint": dict(state["coordinate"]),
            "routeWaypoints": [
                {
                    **state["coordinate"],
                    "etaS": 0.0,
                    "speedMps": 0.0,
                    "distanceM": 0.0,
                }
            ],
        }

    monkeypatch.setattr(ap, "_plan_lah_enemy_contact_response", _capture)
    cover = {"latitude": 37.83, "longitude": 128.11, "altitude": 650}
    cover_state = pa._state_at_post_attack_cover(
        {"coordinate": {"latitude": 37.83, "longitude": 128.11, "altitude": 1200}, "speed": 70},
        cover,
    )

    plan, enemies, _hold_s = pa._post_attack_cover_prelude(
        aircraft_id=2,
        current_state=cover_state,
        destroyed_target_id=11,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert plan is not None and plan["applied"] is True
    assert len(enemies) == 1
    assert captured["state"]["coordinate"] == cover
    assert captured["state"]["speed"] == 0.0
    assert captured["descriptor"]["enemy_contact"]["enemy_coordinate_count"] == 1


def test_revalidated_cover_route_starts_after_the_planned_descent() -> None:
    descent = {
        "waypointID": 102,
        "nextWaypointID": 0,
        "coordinate": {"latitude": 37.83, "longitude": 128.11, "altitude": 650},
        "speed": 30.0,
        "eta": 10,
        "hovering": {"time": 10},
        "attack": {"targetID": 0, "weaponType": 0},
    }
    new_cover = {"latitude": 37.831, "longitude": 128.112, "altitude": 640}
    plan = {
        "applied": True,
        "endpoint": dict(new_cover),
        "routeWaypoints": [
            {**descent["coordinate"], "etaS": 0.0, "speedMps": 0.0, "distanceM": 0.0},
            {**new_cover, "etaS": 8.0, "speedMps": 25.0, "distanceM": 210.0},
        ],
    }

    route, applied, _role = pa._append_post_attack_cover_route(
        [descent],
        aircraft_id=2,
        plan=plan,
        hold_seconds=90,
        emit=lambda _message: None,
        log_prefix="[TEST]",
    )

    assert applied is plan
    assert route[0]["coordinate"] == descent["coordinate"]
    assert route[-1]["coordinate"] == new_cover
    assert route[-1]["hovering"]["time"] == 90


def test_a_distant_contact_does_not_produce_an_unbounded_hold(monkeypatch) -> None:
    _use_entries(monkeypatch, [_entry(12, 39.5, 130.0)])
    monkeypatch.setattr(pa, "_post_attack_uav_states", lambda: [])
    monkeypatch.setattr(ap, "_plan_lah_enemy_contact_response", lambda *a, **k: None)

    _plan, _enemies, hold_s = pa._post_attack_cover_prelude(
        aircraft_id=2,
        current_state=STATE,
        destroyed_target_id=11,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert hold_s == ap.get_runtime_attack_int("lah_wait_hold_max_seconds", 600)


def test_only_uavs_are_offered_to_the_relay_link_check(monkeypatch) -> None:
    """Aircraft 1-3 are the manned flight; they are not relay endpoints."""

    from modules.common import agent_status_snapshot

    monkeypatch.setattr(
        agent_status_snapshot,
        "load_agent_status_snapshot",
        lambda: {
            "agent_states": [
                {"aircraftID": 1, "coordinate": {"latitude": 37.83, "longitude": 128.11}},
                {"aircraftID": 4, "coordinate": {"latitude": 37.85, "longitude": 128.12}},
                {"aircraftID": 5, "coordinate": {"latitude": 37.86, "longitude": 128.13}},
            ]
        },
    )

    states = pa._post_attack_uav_states()

    assert sorted(row["aircraft_id"] for row in states) == [4, 5]


def _resume_route() -> List[Dict[str, Any]]:
    return [
        {
            "waypointID": 41,
            "nextWaypointID": 42,
            "coordinate": {"latitude": 37.880, "longitude": 128.150, "altitude": 700},
            "speed": 40.0,
            "eta": 0,
            "ecf": 0.0,
            "hovering": {"time": 0},
            "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
            "attack": {"targetID": 0, "weaponType": 0},
        },
        {
            "waypointID": 42,
            "nextWaypointID": 0,
            "coordinate": {"latitude": 37.900, "longitude": 128.170, "altitude": 700},
            "speed": 40.0,
            "eta": 60,
            "ecf": 0.0,
            "hovering": {"time": 0},
            "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
            "attack": {"targetID": 0, "weaponType": 0},
        },
    ]


def _certified_plan() -> Dict[str, Any]:
    return {
        "applied": True,
        "routeWaypoints": [
            {
                "latitude": 37.8315,
                "longitude": 128.1145,
                "altitude": 640.0,
                "speedMps": 35.0,
                "etaS": 0.0,
                "distanceM": 0.0,
            },
            {
                "latitude": 37.8340,
                "longitude": 128.1180,
                "altitude": 620.0,
                "speedMps": 35.0,
                "etaS": 12.0,
                "distanceM": 420.0,
            },
        ],
    }


def test_the_cover_route_is_flown_before_the_return_route() -> None:
    spliced, plan, role = pa._splice_post_attack_cover_prelude(
        _resume_route(),
        aircraft_id=2,
        plan=_certified_plan(),
        hold_seconds=90,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert plan is not None
    assert role == "hold"
    assert len(spliced) == 4
    # Concealment first, then the original return route in order.
    assert spliced[1]["coordinate"]["latitude"] == pytest.approx(37.8340)
    assert [wp["coordinate"]["latitude"] for wp in spliced[2:]] == [
        pytest.approx(37.880),
        pytest.approx(37.900),
    ]


def test_the_aircraft_waits_at_the_concealment_point_not_on_the_route() -> None:
    spliced, _plan, _role = pa._splice_post_attack_cover_prelude(
        _resume_route(),
        aircraft_id=2,
        plan=_certified_plan(),
        hold_seconds=90,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert spliced[1]["hovering"]["time"] == 90
    assert all(wp["hovering"]["time"] == 0 for wp in spliced[2:])


def test_the_command_aircraft_splice_is_tagged_as_a_relay() -> None:
    _spliced, _plan, role = pa._splice_post_attack_cover_prelude(
        _resume_route(),
        aircraft_id=1,
        plan=_certified_plan(),
        hold_seconds=90,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert role == "relay"


def test_without_certified_cover_the_aircraft_waits_where_it_is() -> None:
    """Never fly an uncertified detour into a live threat."""

    original = _resume_route()
    spliced, plan, _role = pa._splice_post_attack_cover_prelude(
        original,
        aircraft_id=2,
        plan={"applied": False, "routeWaypoints": []},
        hold_seconds=90,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert plan is None
    assert len(spliced) == len(original)
    assert spliced[0]["hovering"]["time"] == 90
    assert [wp["coordinate"] for wp in spliced] == [wp["coordinate"] for wp in original]


def test_an_existing_longer_hold_is_never_shortened() -> None:
    route = _resume_route()
    route[0]["hovering"] = {"time": 300}

    spliced, _plan, _role = pa._splice_post_attack_cover_prelude(
        route,
        aircraft_id=2,
        plan=None,
        hold_seconds=90,
        emit=lambda _msg: None,
        log_prefix="[TEST]",
    )

    assert spliced[0]["hovering"]["time"] == 300


def test_splicing_cover_in_front_rebases_eta_from_zero() -> None:
    payload = {
        "lahWaypointList": [
            {
                "waypointID": 1,
                "nextWaypointID": 2,
                "coordinate": {"latitude": 37.830, "longitude": 128.113, "altitude": 700},
                "speed": 40.0,
                "eta": 0,
                "ecf": 0.0,
                "hovering": {"time": 60},
            },
            {
                "waypointID": 2,
                "nextWaypointID": 0,
                # Resume waypoint carrying an ETA from the plan it was cut from.
                "coordinate": {"latitude": 37.880, "longitude": 128.150, "altitude": 700},
                "speed": 40.0,
                "eta": 12,
                "ecf": 1.0,
                "hovering": {"time": 0},
            },
        ]
    }

    pa._rebase_post_attack_cover_timing(payload)

    etas = [wp["eta"] for wp in payload["lahWaypointList"]]
    assert etas[0] == 0
    assert etas == sorted(etas)
    # The stale 12 s cannot survive: the leg alone is kilometres long.
    assert etas[1] > 12


def test_rebasing_replaces_the_progress_ramp_with_litres() -> None:
    payload = {
        "lahWaypointList": [
            {
                "waypointID": 1,
                "nextWaypointID": 2,
                "coordinate": {"latitude": 37.830, "longitude": 128.113, "altitude": 700},
                "speed": 40.0,
                "eta": 0,
                "ecf": 0.0,
                "hovering": {"time": 0},
            },
            {
                "waypointID": 2,
                "nextWaypointID": 0,
                "coordinate": {"latitude": 37.880, "longitude": 128.150, "altitude": 700},
                "speed": 40.0,
                "eta": 0,
                "ecf": 1.0,
                "hovering": {"time": 0},
            },
        ]
    }

    pa._rebase_post_attack_cover_timing(payload)

    from modules.common.ecf import ECF_MAX_L, NOMINAL_BURN_L_PER_S

    waypoints = payload["lahWaypointList"]
    assert waypoints[0]["ecf"] == 0.0
    # Per-leg litres, not a 0..1 completion ratio.
    assert waypoints[1]["ecf"] != 1.0
    assert 0.0 < waypoints[1]["ecf"] <= ECF_MAX_L
    assert waypoints[1]["ecf"] == pytest.approx(
        waypoints[1]["eta"] * NOMINAL_BURN_L_PER_S, rel=0.02
    )


def test_the_collab_ecf_normalizer_no_longer_emits_a_completion_ratio() -> None:
    waypoints = [
        {"waypointID": 1, "eta": 0, "ecf": 9.0, "hovering": {"time": 0}},
        {"waypointID": 2, "eta": 100, "ecf": 9.0, "hovering": {"time": 0}},
        {"waypointID": 3, "eta": 200, "ecf": 9.0, "hovering": {"time": 0}},
    ]

    pa._normalize_post_attack_collab_waypoint_ecf(waypoints)

    from modules.common.ecf import NOMINAL_BURN_L_PER_S

    assert waypoints[0]["ecf"] == 0.0
    assert waypoints[-1]["ecf"] != 1.0
    assert waypoints[1]["ecf"] == pytest.approx(100 * NOMINAL_BURN_L_PER_S)
    assert waypoints[2]["ecf"] == pytest.approx(100 * NOMINAL_BURN_L_PER_S)
