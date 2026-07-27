"""A manned attack slot comes back once its attack has been flown.

Observed deadlock: both manned attackers were marked used, so no new attack
could be assigned; the only place that released a slot was the post-attack
rejoin, which only runs after an attack; and it additionally deferred whenever
any UAV anywhere in the plan lineage still had a tracking assignment open. One
UAV holding a track therefore kept both manned aircraft out of the fight for
good, and the same target was re-announced on every detection tick.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.post_attack import pipeline as pa


def _flight_path(path_id: int, *, target_id: int) -> dict[str, Any]:
    """A manned path; target_id 0 means the attack has been replanned away."""

    return {
        "pathID": path_id,
        "lahWaypointList": [
            {
                "waypointID": path_id * 10 + index,
                "coordinate": {"latitude": 37.9, "longitude": 127.3, "altitude": 900},
                "attack": {"targetID": target_id if index == 1 else 0, "weaponType": 2},
            }
            for index in range(3)
        ],
    }


@pytest.fixture()
def plan_db(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """A MissionPlan plus FlightPath tree the release logic can read."""

    (tmp_path / "MissionPlan").mkdir()
    (tmp_path / "IndividualMissionPlan").mkdir()
    (tmp_path / "FlightPath").mkdir()

    def _write(*, attacking: dict[int, int | list[int]]) -> int:
        plan_id = 700000099
        aircraft_list = []
        for aircraft_id in (2, 3):
            raw_targets = attacking.get(aircraft_id, 0)
            target_ids = list(raw_targets) if isinstance(raw_targets, list) else [int(raw_targets)]
            missions = []
            for offset, target_id in enumerate(target_ids):
                path_id = 200000000 + aircraft_id * 10 + offset
                (tmp_path / "FlightPath" / f"{path_id}.json").write_text(
                    json.dumps(_flight_path(path_id, target_id=int(target_id))),
                    encoding="utf-8",
                )
                missions.append({"pathID": path_id})
            imp_id = 800000000 + aircraft_id
            (tmp_path / "IndividualMissionPlan" / f"{imp_id}.json").write_text(
                json.dumps(
                    {
                        "individualMissionPackageID": imp_id,
                        "individualMissionList": missions,
                    }
                ),
                encoding="utf-8",
            )
            aircraft_list.append(
                {"aircraftID": aircraft_id, "individualMissionPackageID": imp_id}
            )
        (tmp_path / "MissionPlan" / f"{plan_id}.json").write_text(
            json.dumps({"missionPlanID": plan_id, "aircraftList": aircraft_list}),
            encoding="utf-8",
        )
        return plan_id

    monkeypatch.setattr(
        pa.db_paths,
        "get_db_subpath",
        lambda kind, name: tmp_path / kind / name,
    )
    return _write


def test_an_aircraft_with_no_attack_left_is_free(plan_db) -> None:
    """The reported state: both manned plans are back to conceal-and-hold."""

    plan_id = plan_db(attacking={})

    assert pa._manned_aircraft_still_attacking(plan_id) == set()


def test_an_aircraft_still_holding_an_attack_stays_busy(plan_db) -> None:
    plan_id = plan_db(attacking={3: 8})

    assert pa._manned_aircraft_still_attacking(plan_id) == {3}


def test_an_aircraft_with_a_second_sequential_attack_stays_busy(plan_db) -> None:
    plan_id = plan_db(attacking={3: [0, 9]})

    assert pa._manned_aircraft_still_attacking(plan_id) == {3}


def test_closing_first_target_keeps_second_target_live(monkeypatch) -> None:
    missions = [{"pathID": 31}, {"pathID": 32}]
    paths = {
        31: _flight_path(31, target_id=10),
        32: _flight_path(32, target_id=9),
    }
    monkeypatch.setattr(
        pa,
        "_load_path_payload",
        lambda path_id, **_kwargs: paths.get(int(path_id)),
    )

    remaining = pa._remaining_lah_live_attack_target_ids(
        missions,
        start_index=0,
        removed_target_ids={10},
    )

    assert remaining == {9}


def test_closing_last_target_leaves_no_attack_live(monkeypatch) -> None:
    missions = [{"pathID": 31}, {"pathID": 32}]
    paths = {
        # The first branch has already been rewritten to descent/cover only.
        31: _flight_path(31, target_id=0),
        32: _flight_path(32, target_id=9),
    }
    monkeypatch.setattr(
        pa,
        "_load_path_payload",
        lambda path_id, **_kwargs: paths.get(int(path_id)),
    )

    remaining = pa._remaining_lah_live_attack_target_ids(
        missions,
        start_index=0,
        removed_target_ids={9},
    )

    assert remaining == set()


def test_another_aircraft_attacking_does_not_hold_this_one(plan_db, monkeypatch) -> None:
    """LAH3 engaged must not keep LAH2 out of the next engagement."""

    plan_id = plan_db(attacking={3: 8})
    released: list[tuple[int, list[int]]] = []
    monkeypatch.setattr(
        pa,
        "release_manned_used",
        lambda package_id, aircraft_ids: released.append((package_id, list(aircraft_ids)))
        or list(aircraft_ids),
    )

    out = pa._release_attack_slots_if_tracking_closed(
        input_package_id=3, current_plan_id=plan_id, emit=lambda _message: None
    )

    assert out == [2]
    assert released == [(3, [2])]


def test_a_finished_engagement_releases_every_slot(plan_db, monkeypatch) -> None:
    plan_id = plan_db(attacking={})
    monkeypatch.setattr(
        pa, "release_manned_used", lambda _package_id, aircraft_ids: list(aircraft_ids)
    )

    out = pa._release_attack_slots_if_tracking_closed(
        input_package_id=3, current_plan_id=plan_id, emit=lambda _message: None
    )

    assert sorted(out) == [2, 3]


def test_an_unreadable_plan_never_strands_the_slots(monkeypatch) -> None:
    """Failing to read the plan must not re-create the deadlock."""

    monkeypatch.setattr(pa, "_manned_aircraft_still_attacking", lambda _plan_id: set())
    monkeypatch.setattr(
        pa, "release_manned_used", lambda _package_id, aircraft_ids: list(aircraft_ids)
    )

    out = pa._release_attack_slots_if_tracking_closed(
        input_package_id=3, current_plan_id=None, emit=lambda _message: None
    )

    assert sorted(out) == [2, 3]
