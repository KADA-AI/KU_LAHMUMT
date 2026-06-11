from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    os.environ.setdefault("REPLAN_ID_ALLOCATOR_TIMING", "0")
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


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def expect_path(label: str, actual: object, expected: Path) -> None:
    actual_path = Path(actual)
    if actual_path != expected:
        fail(f"{label} changed: expected {expected}, got {actual_path}")


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        fail(f"expected JSON artifact missing: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        fail(f"JSON artifact is not an object: {path}")
    return payload


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    path = PROJECT_ROOT / rel_path
    text = path.read_text(encoding="utf-8", errors="ignore")
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing source markers: {missing!r}")


@contextmanager
def patched_db_root(db_root: Path) -> Iterator[None]:
    from modules.common import db_paths

    original_active = db_paths.get_active_db_root
    original_subpath = db_paths.get_db_subpath
    original_bootstrap = getattr(db_paths, "bootstrap_db_root", None)
    db_paths.get_active_db_root = lambda: db_root  # type: ignore[assignment]
    db_paths.get_db_subpath = lambda *parts: db_root.joinpath(*(str(part) for part in parts))  # type: ignore[assignment]
    if original_bootstrap is not None:
        db_paths.bootstrap_db_root = lambda *args, **kwargs: db_root  # type: ignore[assignment]
    try:
        yield
    finally:
        db_paths.get_active_db_root = original_active  # type: ignore[assignment]
        db_paths.get_db_subpath = original_subpath  # type: ignore[assignment]
        if original_bootstrap is not None:
            db_paths.bootstrap_db_root = original_bootstrap  # type: ignore[assignment]


def with_temp_db(callback: Callable[[Path], None], *, prefix: str) -> None:
    tmp_root = Path(tempfile.mkdtemp(prefix=prefix))
    try:
        db_root = tmp_root / "scenario" / "SBC3"
        with patched_db_root(db_root):
            callback(db_root)
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)


def check_static_id_manifest() -> None:
    allocator = importlib.import_module(
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
    )
    id_allocator_0202 = importlib.import_module(
        "modules.mission_planning.MissionPlanner.data_def.id_allocator_0202"
    )
    d0301 = importlib.import_module(
        "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0301"
    )

    data_def_dir = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "data_def"
    expect_path("legacy id_tracker path", allocator._LEGACY_STORE, data_def_dir / "id_tracker.json")
    expect_path("0202 id tracker path", id_allocator_0202._STORE, data_def_dir / "id_tracker_0202.json")
    expect_equal("0202 base state", id_allocator_0202._BASE_STATE, {"individualMissionID": 950_000_000})
    expect_path("0301 counter file path", d0301._COUNTER_FILE, data_def_dir / "_id_counters.json")

    assert_source_contains(
        "modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py",
        '_ID_COUNTER_FILE = os.path.join(os.path.dirname(__file__), "_id_counters.json")',
    )
    assert_source_contains(
        "run.py",
        'tracker_path = data_def_dir / "id_tracker.json"',
        'tracker_0202_path = data_def_dir / "id_tracker_0202.json"',
        'counters_path = data_def_dir / "_id_counters.json"',
        'for fname in ("path_usage.json", "waypoint_usage.json")',
        'store = Path(id_allocator.__file__).resolve().parent / "id_tracker.json"',
        'store_0202 = Path(id_allocator_0202.__file__).resolve().parent / "id_tracker_0202.json"',
        'counter_file = Path(id_allocator.__file__).resolve().parent / "_id_counters.json"',
    )


def check_id_allocator_artifacts(db_root: Path) -> None:
    allocator = importlib.import_module(
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
    )

    dss_dir = db_root / "DSS_Internal"
    expect_path("active id_tracker path", allocator._resolve_store_path(), dss_dir / "id_tracker.json")

    plan_ids = allocator.reserve_mission_plan_ids(1)
    imp_ids = allocator.reserve_imp_ids(1)
    individual_ids = allocator.reserve_individual_mission_ids(1)
    path_ids = allocator.reserve_path_ids(2, 2)
    waypoint_start = allocator.reserve_waypoint_block(2)
    allocator.mark_waypoint_files_written(77)

    expect_equal("reserved missionPlanIDs", plan_ids, [700_000_001])
    expect_equal("reserved IMP IDs", imp_ids, [800_000_001])
    expect_equal("reserved individual mission IDs", individual_ids, [900_000_001])
    expect_equal("reserved path IDs", path_ids, [200_000_001, 200_000_002])
    expect_equal("reserved waypoint start", waypoint_start, 50)

    store = dss_dir / "id_tracker.json"
    store_payload = load_json(store)
    expect_equal("stored missionPlanID high water", store_payload.get("missionPlanID"), 700_000_001)
    expect_equal("stored IMP high water", store_payload.get("individualMissionPackage"), 800_000_001)
    expect_equal("stored individual mission high water", store_payload.get("individualMission"), 900_000_001)
    expect_true("volatile waypoint is not persisted in id_tracker", "waypoint" not in store_payload)
    path_map = store_payload.get("pathID")
    expect_true("id_tracker pathID map", isinstance(path_map, dict))
    expect_equal("stored aircraft 2 path high water", path_map.get("2"), 200_000_002)
    expect_true("id_tracker lock sidecar exists", store.with_name("id_tracker.json.lock").exists())

    path_usage = load_json(dss_dir / "path_usage.json")
    expect_equal("path usage aircraft 2", path_usage.get("aircraft", {}).get("2"), 200_000_002)
    expect_true("path usage timestamp", isinstance(path_usage.get("updated_at"), str))
    expect_true("path usage tmp cleanup", not list(dss_dir.glob("path_usage.tmp")))

    waypoint_usage = load_json(dss_dir / "waypoint_usage.json")
    expect_equal("waypoint usage high water", waypoint_usage.get("last_waypoint_id"), 77)
    expect_true("waypoint usage timestamp", isinstance(waypoint_usage.get("updated_at"), str))
    signature = waypoint_usage.get("flightPathSignature")
    expect_true("waypoint usage signature object", isinstance(signature, dict))
    expect_equal("waypoint signature version", signature.get("version"), 1)
    expect_equal("waypoint signature path", signature.get("path"), str(db_root / "FlightPath"))
    expect_true("waypoint usage tmp cleanup", not list(dss_dir.glob("waypoint_usage.tmp")))


def check_attack_assignment_state(db_root: Path) -> None:
    module = importlib.import_module("modules.mission_planning.runtime.state.attack_assignment")
    dss_dir = db_root / "DSS_Internal"
    path = dss_dir / "attack_assignment_state.json"

    expect_equal("attack assignment filename", module._STATE_FILENAME, "attack_assignment_state.json")
    module.set_last_assigned_manned_id(2)
    module.mark_manned_used(1001, 2)
    module.set_pending_manned_assignments(7001, 1001, [2, 3, 3])
    changed = module.defer_attack_targets(
        1001,
        7001,
        [
            {
                "targetId": "9",
                "watcher_id": 5,
                "coordinate": {"latitude": 37.1, "longitude": 127.2, "altitude": 1000},
            }
        ],
        now_ms=123456,
        reason="smoke",
    )
    expect_equal("deferred attack target IDs", changed, [9])

    payload = load_json(path)
    expect_equal("last manned key", payload.get("last_manned_aircraft_id"), 2)
    expect_equal("used manned map", payload.get("used_manned_by_input_package", {}).get("1001"), [2])
    pending = payload.get("pending_manned_by_plan_id", {}).get("7001")
    expect_equal("pending mission plan ID", pending.get("mission_plan_id"), 7001)
    expect_equal("pending input package ID", pending.get("input_package_id"), 1001)
    expect_equal("pending aircraft IDs", pending.get("aircraft_ids"), [2, 3])
    deferred = payload.get("deferred_attack_targets_by_input_package", {}).get("1001", {}).get("9")
    expect_equal("deferred target ID", deferred.get("targetID"), 9)
    expect_equal("deferred watcher ID", deferred.get("watcherID"), 5)
    expect_equal("deferred source plan ID", deferred.get("sourceMissionPlanID"), 7001)
    expect_equal("deferred reason", deferred.get("deferredReason"), "smoke")

    expect_equal("committed pending manned IDs", module.commit_pending_manned_assignments(7001), [2, 3])
    payload = load_json(path)
    expect_true("pending map removed after commit", "pending_manned_by_plan_id" not in payload)
    expect_equal("committed used manned map", payload.get("used_manned_by_input_package", {}).get("1001"), [2, 3])


def check_attack_tracking_state(db_root: Path) -> None:
    module = importlib.import_module("modules.mission_planning.runtime.state.attack_tracking")
    dss_dir = db_root / "DSS_Internal"
    path = dss_dir / "attack_tracking_state.json"

    expect_equal("attack tracking filename", module._STATE_FILENAME, "attack_tracking_state.json")
    module.register_tracking_assignment(
        aircraft_id=4,
        source_plan_id=7001,
        attack_plan_id=7002,
        current_input_mission_id=11,
        original_path_id=400_000_001,
        original_individual_mission_id=900_000_001,
        original_current_waypoint_id=55,
        original_coordinate={"lat": 37.1, "lon": 127.2, "alt": 1000},
        tracking_path_id=400_000_002,
        tracking_individual_mission_id=900_000_002,
        resume_path_id=400_000_003,
        resume_individual_mission_id=900_000_003,
        target_id=9,
    )

    payload = load_json(path)
    entry = payload.get("assignments", {}).get("4")
    expect_equal("attack assignment active", entry.get("active"), True)
    expect_equal("attack assignment source plan", entry.get("source_plan_id"), 7001)
    expect_equal("attack tracking path", entry.get("tracking_path_id"), 400_000_002)
    expect_equal("attack auto tracking default", entry.get("auto_tracking_engaged"), False)
    expect_equal("attack coordinate latitude", entry.get("last_nonzero_coordinate", {}).get("latitude"), 37.1)
    expect_true("attack registered_at", isinstance(entry.get("registered_at"), str))

    module.update_from_agent_states(
        [
            {
                "aircraftID": 4,
                "currentWaypointID": {"waypointID": 66},
                "coordinate": {"latitude": 37.3, "longitude": 127.4},
            }
        ]
    )
    entry = load_json(path).get("assignments", {}).get("4")
    expect_equal("attack last nonzero waypoint update", entry.get("last_nonzero_waypoint_id"), 66)
    expect_equal("attack updated coordinate longitude", entry.get("last_nonzero_coordinate", {}).get("longitude"), 127.4)

    module.clear_tracking_assignment(4)
    entry = load_json(path).get("assignments", {}).get("4")
    expect_equal("attack clear active flag", entry.get("active"), False)
    expect_true("attack cleared_at", isinstance(entry.get("cleared_at"), str))
    expect_true("attack tracking tmp cleanup", not list(dss_dir.glob("attack_tracking_state.json.*.tmp")))


def check_prior_tracking_state(db_root: Path) -> None:
    module = importlib.import_module("modules.mission_planning.runtime.state.prior_tracking")
    dss_dir = db_root / "DSS_Internal"
    path = dss_dir / "prior_tracking_state.json"

    expect_equal("prior tracking filename", module._STATE_FILENAME, "prior_tracking_state.json")
    module.register_prior_assignment(
        aircraft_id=5,
        source_plan_id=7101,
        prior_plan_id=7102,
        current_input_mission_id=21,
        original_path_id=500_000_001,
        original_individual_mission_id=900_000_011,
        original_current_waypoint_id=77,
        original_coordinate={"Latitude": 37.5, "Longitude": 127.5, "Altitude": 900},
        prior_path_id=500_000_002,
        prior_individual_mission_id=900_000_012,
        prior_waypoint_ids=[10, "11", 10, 0],
        resume_path_id=500_000_003,
        resume_individual_mission_id=900_000_013,
        resume_first_waypoint_id=12,
        resume_first_active_waypoint_id=13,
        resume_waypoint_ids=[12, 13, 13, "bad"],
        prior_mission_id=31,
        mission_type=2,
        target_id=9,
        target_coordinate={"latitude": 37.9, "longitude": 127.9},
    )

    payload = load_json(path)
    entry = payload.get("assignments", {}).get("5")
    expect_equal("prior assignment active", entry.get("active"), True)
    expect_equal("prior source plan", entry.get("source_plan_id"), 7101)
    expect_equal("prior path ID", entry.get("prior_path_id"), 500_000_002)
    expect_equal("prior waypoint IDs normalized", entry.get("prior_waypoint_ids"), [10, 11])
    expect_equal("prior resume waypoint IDs normalized", entry.get("resume_waypoint_ids"), [12, 13])
    expect_equal("prior coordinate altitude", entry.get("original_coordinate", {}).get("altitude"), 900.0)
    expect_true("prior registered_at", isinstance(entry.get("registered_at"), str))

    module.mark_prior_handoff(
        5,
        handoff_waypoint_id=88,
        handoff_coordinate={"lat": 37.6, "lon": 127.6},
    )
    entry = load_json(path).get("assignments", {}).get("5")
    expect_equal("prior handoff waypoint", entry.get("handoff_waypoint_id"), 88)
    expect_equal("prior handoff longitude", entry.get("handoff_coordinate", {}).get("longitude"), 127.6)

    module.clear_prior_assignment(5, reason="smoke")
    entry = load_json(path).get("assignments", {}).get("5")
    expect_equal("prior clear active flag", entry.get("active"), False)
    expect_equal("prior clear reason", entry.get("clear_reason"), "smoke")
    expect_true("prior cleared_at", isinstance(entry.get("cleared_at"), str))
    expect_true("prior tracking tmp cleanup", not list(dss_dir.glob("prior_tracking_state.json.*.tmp")))


def check_runtime_state_artifacts(db_root: Path) -> None:
    check_attack_assignment_state(db_root)
    check_attack_tracking_state(db_root)
    check_prior_tracking_state(db_root)


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke ID/state JSON artifact manifest.")
    parser.parse_args()

    try:
        configure_import_paths()
        with_temp_db(lambda _db_root: check_static_id_manifest(), prefix="mp_id_static_")
        with_temp_db(check_id_allocator_artifacts, prefix="mp_id_artifacts_")
        with_temp_db(check_runtime_state_artifacts, prefix="mp_state_artifacts_")
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("ID/state JSON artifact manifest smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
