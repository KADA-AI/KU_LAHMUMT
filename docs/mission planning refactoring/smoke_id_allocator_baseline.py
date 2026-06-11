from __future__ import annotations

import argparse
import importlib
import inspect
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ALLOCATOR_MODULE = "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
ABSOLUTE_WRAPPER = "modules.mission_planning.MissionPlanner.data_def.id_allocator"
BARE_WRAPPER = "data_def.id_allocator"

EXPECTED_BASE = {
    "missionPlanID": 700_000_001,
    "individualMissionPackage": 800_000_001,
    "individualMission": 900_000_001,
    "pathID": {
        1: 100_000_001,
        2: 200_000_001,
        3: 300_000_001,
        4: 400_000_001,
        5: 500_000_001,
        6: 600_000_001,
    },
    "waypoint": 50,
}

EXPECTED_SIGNATURES = {
    "next_mission_plan_id": "()",
    "next_imp_id": "()",
    "next_individual_mission_id": "()",
    "next_path_id": "(aircraft_id: int)",
    "next_waypoint_id": "()",
    "reserve_mission_plan_ids": "(count: int)",
    "reserve_imp_ids": "(count: int)",
    "reserve_individual_mission_ids": "(count: int)",
    "reserve_path_ids": "(aircraft_id: int, count: int)",
    "reserve_path_id_blocks": "(count_by_aircraft: dict[int, int] | dict[str, int])",
    "reserve_replan_id_bundle": "(*, path_count_by_aircraft: dict[int, int] | dict[str, int] | None = None, imp_count: int = 0, individual_mission_count: int = 0)",
    "reserve_waypoint_block": "(count: int) -> int",
    "reserve_waypoint_blocks": "(counts: list[int] | tuple[int, ...]) -> list[tuple[int, int]]",
    "mark_waypoint_files_written": "(max_waypoint_id: int | None = None) -> None",
}


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def fail(message: str) -> None:
    raise RuntimeError(message)


def check_paths(allocator: Any) -> None:
    expected_legacy_store = (
        PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "data_def" / "id_tracker.json"
    )
    if Path(allocator._LEGACY_STORE) != expected_legacy_store:
        fail(f"legacy id allocator store changed: {allocator._LEGACY_STORE!r}")
    if allocator.BASE != EXPECTED_BASE:
        fail(f"id allocator BASE changed: {allocator.BASE!r}")

    wrapper = importlib.import_module(ABSOLUTE_WRAPPER)
    if Path(wrapper.__file__) != expected_legacy_store.with_name("id_allocator.py"):
        fail(f"id allocator absolute wrapper __file__ changed: {wrapper.__file__!r}")
    if (PROJECT_ROOT / "modules" / "mission_planning" / "engine" / "mission_generation" / "id_allocation" / "id_tracker.json").exists():
        fail("id_tracker.json must not live beside the canonical engine allocator")

    resolved = Path(allocator._resolve_store_path())
    if resolved.name != "id_tracker.json" or resolved.parent.name != "DSS_Internal":
        fail(f"active id allocator store no longer resolves under DSS_Internal: {resolved}")


def check_signatures(allocator: Any) -> None:
    for attr, expected in EXPECTED_SIGNATURES.items():
        value = getattr(allocator, attr, None)
        if not callable(value):
            fail(f"id allocator public API missing callable {attr}")
        actual = str(inspect.signature(value))
        if actual != expected:
            fail(f"id allocator signature changed for {attr}: {actual!r} != {expected!r}")


def check_wrapper_identity(allocator: Any) -> None:
    absolute = importlib.import_module(ABSOLUTE_WRAPPER)
    bare = importlib.import_module(BARE_WRAPPER)
    for attr in EXPECTED_SIGNATURES:
        if getattr(absolute, attr) is not getattr(allocator, attr):
            fail(f"absolute id_allocator wrapper identity split: {attr}")
        if getattr(bare, attr) is not getattr(allocator, attr):
            fail(f"bare id_allocator wrapper identity split: {attr}")
    if absolute.BASE is not allocator.BASE:
        fail("absolute id_allocator wrapper BASE identity split")
    if bare.BASE is not allocator.BASE:
        fail("bare id_allocator wrapper BASE identity split")


def check_isolated_reserve_behavior(allocator: Any) -> None:
    originals = {
        "_STORE": allocator._STORE,
        "_state": allocator._state,
        "_volatile_counters": allocator._volatile_counters,
        "_sync_active_db_scope": allocator._sync_active_db_scope,
        "_record_path_usage_many": allocator._record_path_usage_many,
        "_record_waypoint_usage": allocator._record_waypoint_usage,
        "_refresh_waypoint_counter": allocator._refresh_waypoint_counter,
        "_emit_id_metric": allocator._emit_id_metric,
    }
    with tempfile.TemporaryDirectory(prefix="id_allocator_baseline_") as tmp:
        store = Path(tmp) / "DSS_Internal" / "id_tracker.json"
        try:
            allocator._STORE = store
            allocator._state = {}
            allocator._volatile_counters = {
                key: allocator.BASE[key] - 1
                for key in getattr(allocator, "VOLATILE_KEYS", set())
            }
            allocator._sync_active_db_scope = lambda: None
            allocator._record_path_usage_many = lambda _updates: None
            allocator._record_waypoint_usage = lambda _value, **_kwargs: None
            allocator._refresh_waypoint_counter = lambda **_kwargs: None
            allocator._emit_id_metric = lambda *_args, **_kwargs: None

            if allocator.reserve_mission_plan_ids(2) != [700_000_001, 700_000_002]:
                fail("reserve_mission_plan_ids baseline changed")
            if allocator.reserve_imp_ids(2) != [800_000_001, 800_000_002]:
                fail("reserve_imp_ids baseline changed")
            if allocator.reserve_individual_mission_ids(1) != [900_000_001]:
                fail("reserve_individual_mission_ids baseline changed")
            if allocator.reserve_path_ids(1, 2) != [100_000_001, 100_000_002]:
                fail("reserve_path_ids baseline changed")

            path_blocks = allocator.reserve_path_id_blocks({2: 2, "3": 1, 4: 0})
            if path_blocks != {2: [200_000_001, 200_000_002], 3: [300_000_001]}:
                fail(f"reserve_path_id_blocks baseline changed: {path_blocks!r}")
            if allocator.reserve_path_id_blocks({}) != {}:
                fail("reserve_path_id_blocks empty baseline changed")

            empty_bundle = allocator.reserve_replan_id_bundle()
            if empty_bundle != {
                "pathID": {},
                "individualMissionPackage": [],
                "individualMission": [],
            }:
                fail(f"reserve_replan_id_bundle empty baseline changed: {empty_bundle!r}")

            bundle = allocator.reserve_replan_id_bundle(
                path_count_by_aircraft={4: 2},
                imp_count=1,
                individual_mission_count=2,
            )
            if bundle != {
                "pathID": {4: [400_000_001, 400_000_002]},
                "individualMissionPackage": [800_000_003],
                "individualMission": [900_000_002, 900_000_003],
            }:
                fail(f"reserve_replan_id_bundle baseline changed: {bundle!r}")

            if allocator.reserve_waypoint_block(3) != 50:
                fail("reserve_waypoint_block baseline changed")
            if allocator.reserve_waypoint_blocks([2, 0, "1"]) != [(53, 54), (55, 55)]:
                fail("reserve_waypoint_blocks baseline changed")
            try:
                allocator.reserve_path_ids(99, 1)
            except KeyError as exc:
                if exc.args != (99,):
                    fail(f"reserve_path_ids unknown aircraft KeyError changed: {exc!r}")
            else:
                fail("reserve_path_ids unknown aircraft baseline changed")
            try:
                allocator.reserve_waypoint_block(0)
            except ValueError:
                pass
            else:
                fail("reserve_waypoint_block zero-count baseline changed")

            if not store.exists():
                fail("isolated id_tracker.json was not written")
            stored = json.loads(store.read_text(encoding="utf-8"))
            expected_stored = {
                "missionPlanID": 700_000_002,
                "individualMissionPackage": 800_000_003,
                "individualMission": 900_000_003,
                "pathID": {
                    "1": 100_000_002,
                    "2": 200_000_002,
                    "3": 300_000_001,
                    "4": 400_000_002,
                },
            }
            for key, expected in expected_stored.items():
                if stored.get(key) != expected:
                    fail(f"isolated id_tracker stored {key} changed: {stored.get(key)!r} != {expected!r}")
            if store.with_name("id_tracker.json.lock").exists() is False:
                fail("isolated id_tracker lock file was not created")
        finally:
            for name, value in originals.items():
                setattr(allocator, name, value)


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke ID allocator counter and reserve API baseline.")
    parser.parse_args()

    try:
        configure_import_paths()
        allocator = importlib.import_module(ALLOCATOR_MODULE)
        check_paths(allocator)
        check_signatures(allocator)
        check_wrapper_identity(allocator)
        check_isolated_reserve_behavior(allocator)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("id allocator counter/reserve baseline smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
