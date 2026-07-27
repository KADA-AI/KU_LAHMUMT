"""An exclusion plan is defined by having no tracking left in it.

The per-aircraft detach only fires when the tracking assignment names the plan
being excluded from, so tracking inherited from an earlier replan survived into
a plan whose entire purpose is to carry none. Observed live: exclusion plan
700000008 kept UAV5 tracking target 7 while UAV4/UAV6 were cleared.
"""

from __future__ import annotations

import json
from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap


def _write(tmp_path, sub: str, name: int, payload: dict[str, Any]):
    d = tmp_path / sub
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{name}.json").write_text(json.dumps(payload), encoding="utf-8")


def _path(pid: int, *, tracking: bool) -> dict[str, Any]:
    filming = {"operationMode": 3 if tracking else 2, "fieldOfView": 4.7}
    if not tracking:
        filming["lineSearch"] = {"coordinateList": [{"latitude": 37.0, "longitude": 128.0}]}
    return {
        "pathID": pid,
        "waypointList": [
            {"waypointID": pid * 10, "nextWaypointID": 0, "filmingProperty": filming}
        ],
    }


@pytest.fixture
def db(tmp_path, monkeypatch):
    monkeypatch.setattr(
        ap.db_paths, "get_db_subpath", lambda sub, name: tmp_path / sub / name
    )
    # Bypass any read cache so the fixture files are what gets read.
    monkeypatch.setattr(ap, "read_json_cached", lambda p, kind=None: json.loads(p.read_text(encoding="utf-8")))
    return tmp_path


def _plan_with(db, *, sweep_ids, tracking_ids, aircraft_id=5, imp_id=800):
    missions = []
    for pid in sweep_ids:
        _write(db, "FlightPath", pid, _path(pid, tracking=False))
        missions.append({"individualMissionID": 900 + pid, "pathID": pid})
    for pid in tracking_ids:
        _write(db, "FlightPath", pid, _path(pid, tracking=True))
        missions.append({"individualMissionID": 900 + pid, "pathID": pid})
    _write(db, "IndividualMissionPlan", imp_id, {"individualMissionList": missions})
    return {
        "aircraftList": [
            {"aircraftID": aircraft_id, "individualMissionPackageID": imp_id}
        ]
    }


def _surviving(db, imp_id=800):
    data = json.loads((db / "IndividualMissionPlan" / f"{imp_id}.json").read_text(encoding="utf-8"))
    return [m["pathID"] for m in data["individualMissionList"]]


def test_a_leftover_tracking_mission_is_removed(db) -> None:
    plan = _plan_with(db, sweep_ids=[11, 12], tracking_ids=[13])

    removed = ap._strip_tracking_from_exclusion_plan(plan, emit=lambda _m: None)

    assert [row["pathID"] for row in removed] == [13]
    assert _surviving(db) == [11, 12]


def test_the_remaining_area_sweeps_are_untouched(db) -> None:
    plan = _plan_with(db, sweep_ids=[21, 22, 23], tracking_ids=[24])

    ap._strip_tracking_from_exclusion_plan(plan, emit=lambda _m: None)

    assert _surviving(db) == [21, 22, 23]


def test_a_plan_that_is_already_clean_is_not_rewritten(db) -> None:
    plan = _plan_with(db, sweep_ids=[31, 32], tracking_ids=[])
    before = (db / "IndividualMissionPlan" / "800.json").read_text(encoding="utf-8")

    removed = ap._strip_tracking_from_exclusion_plan(plan, emit=lambda _m: None)

    assert removed == []
    assert (db / "IndividualMissionPlan" / "800.json").read_text(encoding="utf-8") == before


def test_an_aircraft_with_only_tracking_keeps_its_missions(db) -> None:
    """Never hand back an empty package."""

    plan = _plan_with(db, sweep_ids=[], tracking_ids=[41])
    messages: list[str] = []

    removed = ap._strip_tracking_from_exclusion_plan(plan, emit=messages.append)

    assert removed == []
    assert _surviving(db) == [41]
    assert any("only tracking missions" in m for m in messages)


def test_tracking_is_detected_by_the_filming_operation_mode(db) -> None:
    _write(db, "FlightPath", 51, _path(51, tracking=True))
    _write(db, "FlightPath", 52, _path(52, tracking=False))

    assert ap._mission_films_in_tracking_mode({"pathID": 51}) is True
    assert ap._mission_films_in_tracking_mode({"pathID": 52}) is False
    assert ap._mission_films_in_tracking_mode({"pathID": 0}) is False
    assert ap._mission_films_in_tracking_mode({}) is False


def test_the_sweep_runs_on_the_finished_exclusion_plan() -> None:
    """It must be a backstop after the per-aircraft detach, not a replacement."""

    import inspect

    source = inspect.getsource(ap)
    assert "_strip_tracking_from_exclusion_plan(new_plan_data" in source
    # Ahead of validation, so a stripped plan is what gets validated and written.
    strip_at = source.index("_strip_tracking_from_exclusion_plan(new_plan_data")
    validate_at = source.index('scope="attack_exclusion"')
    assert strip_at < validate_at
