from __future__ import annotations

import argparse
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


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        fail(f"expected JSON artifact missing: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        fail(f"JSON artifact is not an object: {path}")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


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
    db_paths.get_active_db_root = lambda: db_root  # type: ignore[assignment]
    db_paths.get_db_subpath = lambda *parts: db_root.joinpath(*(str(part) for part in parts))  # type: ignore[assignment]
    try:
        yield
    finally:
        db_paths.get_active_db_root = original_active  # type: ignore[assignment]
        db_paths.get_db_subpath = original_subpath  # type: ignore[assignment]


def with_temp_db(callback: Callable[[Path], None]) -> None:
    tmp_root = Path(tempfile.mkdtemp(prefix="mp_runtime_db_state_"))
    try:
        db_root = tmp_root / "scenario" / "SBC3"
        with patched_db_root(db_root):
            callback(db_root)
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)


def check_vehicle_status(db_root: Path) -> None:
    from modules.monitoring.utils import vehicle_status
    from modules.mission_planning.MissionPlanner.planning_enhanced import pipeline as enhanced_pipeline

    expect_equal("manned aircraft IDs", vehicle_status.MANNED_AIRCRAFT_IDS, (1, 2, 3))
    expect_equal("unmanned aircraft IDs", vehicle_status.UNMANNED_AIRCRAFT_IDS, (4, 5, 6))

    vehicle_status.write_vehicle_status([4, "2", 6, "bad", 4])
    path = db_root / "VehicleStatus" / "status.json"
    payload = load_json(path)
    expect_equal("VehicleStatus available list", payload.get("available"), [2, 4, 6])
    expect_equal("VehicleStatus manned map", payload.get("manned"), {"1": 0, "2": 1, "3": 0})
    expect_equal("VehicleStatus unmanned map", payload.get("unmanned"), {"4": 1, "5": 0, "6": 1})
    expect_true("VehicleStatus updated_at", isinstance(payload.get("updated_at"), str))
    expect_true("VehicleStatus tmp cleanup", not (path.with_suffix(".tmp")).exists())

    available = enhanced_pipeline._load_vehicle_status_available()
    expect_equal("enhanced VehicleStatus reader", available, {2, 4, 6})


def check_target_info(db_root: Path) -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline
    from modules.mission_planning.replanning.triggers.prior import pipeline as prior_pipeline

    target_path = db_root / "DSS_Internal" / "targetInfo.json"
    write_json(
        target_path,
        {
            "timestamp_ms": 833900001234,
            "targetList": {
                "9-5": {
                    "targetID": 9,
                    "targetType": 2,
                    "coordinate": {"latitude": 37.1, "longitude": 127.2, "altitude": 1000},
                    "isDestroyed": False,
                    "isUsed": 1,
                    "isIgnored": 0,
                    "targetInFrame": True,
                    "threat": 0.8,
                    "firstDetected": 100,
                    "lastUpdated": 120,
                },
                "10": {
                    "targetID": 10,
                    "targetType": 3,
                    "watcher": {"aircraftID": 6},
                    "coordinate": {"lat": 37.3, "lon": 127.4},
                    "isDestroyed": True,
                    "targetInFrame": False,
                },
            },
        },
    )

    entries, error = attack_pipeline._load_target_entries()
    expect_equal("targetInfo load error", error, None)
    expect_equal("targetInfo entry count", len(entries), 2)
    first = entries[0]
    expect_equal("targetInfo key preserved", first.get("key"), "9-5")
    expect_equal("targetInfo target ID", first.get("target_id"), 9)
    expect_equal("targetInfo watcher from key", first.get("watcher_id"), 5)
    expect_equal("targetInfo normalized longitude", first.get("coordinate", {}).get("longitude"), 127.2)
    expect_equal("targetInfo target in frame", first.get("target_in_frame"), True)
    expect_equal("targetInfo raw object preserved", first.get("raw", {}).get("targetID"), 9)

    prior_entry = prior_pipeline._load_target_tracking_entry(10)
    expect_equal("prior targetInfo watcher from nested watcher", prior_entry.get("watcherID"), 6)
    expect_equal("prior targetInfo key", prior_entry.get("_key"), "10")

    write_json(target_path, {"targetList": []})
    entries, error = attack_pipeline._load_target_entries()
    expect_equal("targetInfo invalid entries", entries, [])
    expect_equal("targetInfo invalid error", error, "targetInfo.json lacks targetList")


def check_sweep_progress(db_root: Path) -> None:
    from modules.mission_planning.pipelines import mission_path_trim
    from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack_pipeline

    path = db_root / "DSS_Internal" / "sweep_progress.json"
    write_json(
        path,
        {
            "timestamp_ms": 833900002000,
            "entries": [
                {
                    "path_id": 400_000_001,
                    "aircraft_id": 4,
                    "mission_plan_id": 700_000_001,
                    "input_mission_id": 11,
                    "planned_seconds": 100.0,
                    "elapsed_seconds": 25.0,
                    "seconds_per_point": 5.0,
                    "sweep_point_count": 20,
                    "progress_points": 5,
                    "buffer_seconds": 9.0,
                    "buffer_percent": 10,
                    "buffer_points": 2,
                },
                {"path_id": "bad", "progress_points": 1},
                "ignored",
            ],
        },
    )

    loaded = mission_path_trim.load_sweep_progress()
    expect_equal("sweep progress keys", sorted(loaded), [400_000_001])
    entry = loaded[400_000_001]
    expect_equal("sweep progress points", mission_path_trim.sweep_progress_points(entry), 5)
    expect_equal("sweep estimated buffer points", mission_path_trim.estimate_sweep_buffer_points(entry, 9.0), 6)
    expect_equal("sweep cut points preserves explicit buffer", mission_path_trim.sweep_cut_points(entry), 2)
    expect_equal(
        "post-attack sweep reader",
        post_attack_pipeline._load_sweep_progress_safe(),
        {400_000_001: entry},
    )

    write_json(path, {"entries": {}})
    expect_equal("invalid sweep progress fallback", mission_path_trim.load_sweep_progress(), {})


def check_coverage_progress(db_root: Path) -> None:
    from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack_pipeline

    payload = {
        "timestamp_ms": 833900003000,
        "mission_plan_id": 700_000_001,
        "plan_coverage": {"coverage_percent": 44, "covered_area_m2": 120.5},
        "input_coverage": {"11": {"coverage_percent": 50}},
        "package_coverage": {"800000001": {"coverage_percent": 44}},
        "missions": [
            {
                "aircraft_id": 4,
                "input_id": 11,
                "mission_id": 900_000_001,
                "mission_type": 3,
                "coverage_enabled": True,
                "coverage_percent": 50,
                "covered_area_m2": 120.5,
                "planned_area_m2": 240.0,
                "done": False,
            }
        ],
    }
    write_json(db_root / "DSS_Internal" / "coverage_progress.json", payload)
    loaded = post_attack_pipeline._load_coverage_progress_safe()
    expect_equal("coverage progress payload", loaded, payload)


def check_mission_progress(db_root: Path) -> None:
    from modules.mission_planning.replanning.triggers.prior import pipeline as prior_pipeline

    progress_dir = db_root / "DSS_Internal" / "mission_progress"
    write_json(progress_dir / "progress_old.json", {"missionPlanID": 700_000_001})
    write_json(progress_dir / "progress_new.json", {"missionPlanID": 700_000_002, "aircraftID": 5})
    old_time = 1_700_000_000
    new_time = old_time + 10
    os.utime(progress_dir / "progress_old.json", (old_time, old_time))
    os.utime(progress_dir / "progress_new.json", (new_time, new_time))
    expect_equal("latest mission progress plan ID", prior_pipeline._load_latest_mission_progress_plan_id(), 700_000_002)


def check_source_markers() -> None:
    assert_source_contains(
        "modules/monitoring/utils/vehicle_status.py",
        'status_root = db_paths.get_active_db_root() / "VehicleStatus"',
        'target = status_root / "status.json"',
        '"available": sorted(available)',
        '"manned": {str(aid): int(aid in available) for aid in MANNED_AIRCRAFT_IDS}',
        '"unmanned": {str(aid): int(aid in available) for aid in UNMANNED_AIRCRAFT_IDS}',
    )
    assert_source_contains(
        "modules/sim/runtime/sim_service.py",
        'path = db_paths.get_db_subpath("DSS_Internal", "targetInfo.json")',
        'target_list = data.get("targetList")',
        'merged_entry["isDestroyed"] = True',
    )
    assert_source_contains(
        "modules/monitoring/gui/tabs/monitoring_visualization_tab.py",
        'path = base / "sweep_progress.json"',
        '"entries": list(self._sweep_progress_cache.values())',
        'path = base / "coverage_progress.json"',
        '"plan_coverage": dict(snapshot.get("plan_coverage") or {})',
        '"missions": mission_entries',
    )
    assert_source_contains(
        "modules/mission_planning/replanning/triggers/prior/pipeline.py",
        'progress_dir = db_paths.get_db_subpath("DSS_Internal", "mission_progress")',
        'target_path = db_paths.get_db_subpath("DSS_Internal", "targetInfo.json")',
    )


def check_runtime_db_state_artifacts(db_root: Path) -> None:
    check_vehicle_status(db_root)
    check_target_info(db_root)
    check_sweep_progress(db_root)
    check_coverage_progress(db_root)
    check_mission_progress(db_root)
    check_source_markers()


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke runtime DB state artifact manifest.")
    parser.parse_args()

    try:
        configure_import_paths()
        with_temp_db(check_runtime_db_state_artifacts)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("runtime DB state artifact manifest smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
