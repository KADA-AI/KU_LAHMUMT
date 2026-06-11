from __future__ import annotations

import argparse
import ast
import importlib
import json
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ALLOCATOR_MODULE = "modules.mission_planning.engine.mission_generation.id_allocation.allocator"


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


def fail(message: str) -> None:
    raise RuntimeError(message)


def _fresh_volatile_counters(allocator: Any) -> dict[str, int]:
    return {
        key: int(allocator.BASE[key]) - 1
        for key in getattr(allocator, "VOLATILE_KEYS", set())
    }


def _reset_memory(allocator: Any) -> None:
    allocator._state = {}
    allocator._volatile_counters = _fresh_volatile_counters(allocator)
    allocator._WAYPOINT_SIGNATURE_CACHE = None


@contextmanager
def isolated_allocator(allocator: Any, *, prefix: str) -> Iterator[Path]:
    originals = {
        "_STORE": allocator._STORE,
        "_state": allocator._state,
        "_volatile_counters": allocator._volatile_counters,
        "_WAYPOINT_SIGNATURE_CACHE": getattr(allocator, "_WAYPOINT_SIGNATURE_CACHE", None),
        "_sync_active_db_scope": allocator._sync_active_db_scope,
        "_record_path_usage_many": allocator._record_path_usage_many,
        "_record_waypoint_usage": allocator._record_waypoint_usage,
        "_refresh_waypoint_counter": allocator._refresh_waypoint_counter,
        "_emit_id_metric": allocator._emit_id_metric,
    }
    with tempfile.TemporaryDirectory(prefix=prefix) as tmp:
        store = Path(tmp) / "DSS_Internal" / "id_tracker.json"
        try:
            allocator._STORE = store
            _reset_memory(allocator)
            allocator._sync_active_db_scope = lambda: None
            allocator._record_path_usage_many = lambda _updates: None
            allocator._record_waypoint_usage = lambda _value, **_kwargs: None
            allocator._refresh_waypoint_counter = lambda **_kwargs: None
            allocator._emit_id_metric = lambda *_args, **_kwargs: None
            yield store
        finally:
            for name, value in originals.items():
                setattr(allocator, name, value)


def _assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        fail(f"{label} changed: {actual!r} != {expected!r}")


def _assert_exact_range(values: list[int], start: int, count: int, label: str) -> None:
    actual = sorted(int(value) for value in values)
    expected = list(range(int(start), int(start) + int(count)))
    if actual != expected:
        fail(f"{label} range changed: {actual!r} != {expected!r}")
    if len(actual) != len(set(actual)):
        fail(f"{label} produced duplicate IDs: {actual!r}")


def _assert_contiguous_block(block: list[int], *, label: str) -> None:
    if not block:
        fail(f"{label} returned an empty block")
    expected = list(range(int(block[0]), int(block[0]) + len(block)))
    if list(block) != expected:
        fail(f"{label} block is no longer contiguous: {block!r}")


def _run_parallel(count: int, fn: Callable[[int], Any]) -> list[Any]:
    with ThreadPoolExecutor(max_workers=min(8, max(1, int(count)))) as pool:
        return list(pool.map(fn, range(int(count))))


def _stored_json(store: Path) -> dict[str, Any]:
    if not store.exists():
        fail(f"store was not written: {store}")
    return json.loads(store.read_text(encoding="utf-8"))


def _load_run_reset_namespace(temp_root: Path, active_db_root: Path) -> dict[str, Any]:
    source_path = PROJECT_ROOT / "run.py"
    source = source_path.read_text(encoding="utf-8-sig")
    tree = ast.parse(source, filename=str(source_path))
    selected_nodes: list[ast.stmt] = []
    wanted_names = {
        "hardened_base_ids",
        "_force_reset_id_files",
        "_reset_id_counters",
    }
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {
                target.id
                for target in node.targets
                if isinstance(target, ast.Name)
            }
            if names & wanted_names:
                selected_nodes.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in wanted_names:
            selected_nodes.append(node)

    if len(selected_nodes) != 3:
        fail(f"run.py reset contract extraction changed: {len(selected_nodes)} nodes")

    class FakeDbPaths:
        def bootstrap_db_root(self) -> Path:
            active_db_root.mkdir(parents=True, exist_ok=True)
            return active_db_root

    namespace: dict[str, Any] = {
        "__name__": "run_reset_contract_smoke",
        "PROJECT_ROOT": temp_root,
        "Path": Path,
        "json": json,
        "db_paths": FakeDbPaths(),
    }
    module = ast.Module(body=selected_nodes, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(source_path), "exec"), namespace)
    return namespace


def check_run_reset_contract(allocator: Any) -> None:
    wrapper = importlib.import_module("modules.mission_planning.MissionPlanner.data_def.id_allocator")
    id_allocator_0202 = importlib.import_module(
        "modules.mission_planning.MissionPlanner.data_def.id_allocator_0202"
    )
    originals = {
        "allocator_file": allocator.__file__,
        "wrapper_file": wrapper.__file__,
        "allocator_state": allocator._state,
        "allocator_volatile": allocator._volatile_counters,
        "allocator_waypoint_signature": getattr(allocator, "_WAYPOINT_SIGNATURE_CACHE", None),
        "allocator_0202_file": id_allocator_0202.__file__,
        "allocator_0202_state": getattr(id_allocator_0202, "_STATE", None),
    }
    with tempfile.TemporaryDirectory(prefix="id_allocator_run_reset_") as tmp:
        temp_root = Path(tmp) / "project"
        data_def_dir = temp_root / "modules" / "mission_planning" / "MissionPlanner" / "data_def"
        active_db_root = Path(tmp) / "active_db"
        dss_dir = active_db_root / "DSS_Internal"
        data_def_dir.mkdir(parents=True, exist_ok=True)
        dss_dir.mkdir(parents=True, exist_ok=True)
        (dss_dir / "path_usage.json").write_text("{}", encoding="utf-8")
        (dss_dir / "waypoint_usage.json").write_text("{}", encoding="utf-8")

        try:
            allocator.__file__ = str(data_def_dir / "id_allocator.py")
            wrapper.__file__ = str(data_def_dir / "id_allocator.py")
            allocator._state = {"missionPlanID": 999}
            allocator._volatile_counters = {"waypoint": 999}
            id_allocator_0202.__file__ = str(data_def_dir / "id_allocator_0202.py")
            if hasattr(id_allocator_0202, "_STATE"):
                id_allocator_0202._STATE = {"individualMissionID": 999}  # type: ignore[attr-defined]

            namespace = _load_run_reset_namespace(temp_root, active_db_root)
            hardened_base_ids = namespace["hardened_base_ids"]
            _assert_equal(
                hardened_base_ids,
                {
                    "missionPlanID": 700_000_000,
                    "individualMissionPackage": 800_000_000,
                    "individualMission": 900_000_000,
                    "pathID": {
                        1: 100_000_000,
                        2: 200_000_000,
                        3: 300_000_000,
                        4: 400_000_000,
                        5: 500_000_000,
                        6: 600_000_000,
                    },
                },
                "run.py hardened_base_ids",
            )

            namespace["_force_reset_id_files"]()
            tracker = json.loads((data_def_dir / "id_tracker.json").read_text(encoding="utf-8"))
            tracker_0202 = json.loads((data_def_dir / "id_tracker_0202.json").read_text(encoding="utf-8"))
            counters = json.loads((data_def_dir / "_id_counters.json").read_text(encoding="utf-8"))
            hardened_file_payload = {
                "missionPlanID": 700_000_000,
                "individualMissionPackage": 800_000_000,
                "individualMission": 900_000_000,
                "pathID": {
                    "1": 100_000_000,
                    "2": 200_000_000,
                    "3": 300_000_000,
                    "4": 400_000_000,
                    "5": 500_000_000,
                    "6": 600_000_000,
                },
            }
            _assert_equal(tracker, hardened_file_payload, "force reset id_tracker payload")
            _assert_equal(tracker_0202, {"individualMissionID": 950_000_000}, "force reset 0202 payload")
            _assert_equal(
                counters,
                {"missionPlanID": 700_000_000, "impPackageID": 800_000_000},
                "force reset _id_counters payload",
            )
            if (dss_dir / "path_usage.json").exists() or (dss_dir / "waypoint_usage.json").exists():
                fail("force reset no longer clears active DB path/waypoint usage artifacts")
            if (dss_dir / "id_tracker.json").exists():
                fail("force reset unexpectedly writes active DB id_tracker.json")

            allocator._state = {"missionPlanID": 123}
            allocator._volatile_counters = {"waypoint": 123}
            if hasattr(id_allocator_0202, "_STATE"):
                id_allocator_0202._STATE = {"individualMissionID": 123}  # type: ignore[attr-defined]
            namespace["_reset_id_counters"]()

            reset_tracker = json.loads((data_def_dir / "id_tracker.json").read_text(encoding="utf-8"))
            expected_state = {
                "missionPlanID": 700_000_000,
                "individualMissionPackage": 800_000_000,
                "individualMission": 900_000_000,
                "pathID": {
                    1: 100_000_000,
                    2: 200_000_000,
                    3: 300_000_000,
                    4: 400_000_000,
                    5: 500_000_000,
                    6: 600_000_000,
                },
            }
            expected_tracker = {
                "missionPlanID": 700_000_000,
                "individualMissionPackage": 800_000_000,
                "individualMission": 900_000_000,
                "pathID": {
                    "1": 100_000_000,
                    "2": 200_000_000,
                    "3": 300_000_000,
                    "4": 400_000_000,
                    "5": 500_000_000,
                    "6": 600_000_000,
                },
            }
            _assert_equal(reset_tracker, expected_tracker, "cold reset id_tracker payload")
            _assert_equal(allocator._state, expected_state, "cold reset allocator _state")
            _assert_equal(allocator._volatile_counters, {"waypoint": 49}, "cold reset volatile counters")
            reset_0202 = json.loads((data_def_dir / "id_tracker_0202.json").read_text(encoding="utf-8"))
            _assert_equal(reset_0202, {"individualMissionID": 950_000_000}, "cold reset 0202 tracker")
            if hasattr(id_allocator_0202, "_STATE"):
                _assert_equal(
                    id_allocator_0202._STATE,  # type: ignore[attr-defined]
                    {"individualMissionID": 950_000_000},
                    "cold reset 0202 in-memory state",
                )
            reset_counters = json.loads((data_def_dir / "_id_counters.json").read_text(encoding="utf-8"))
            _assert_equal(
                reset_counters,
                {"missionPlanID": 700_000_001, "impPackageID": 800_000_001},
                "cold reset _id_counters payload",
            )
        finally:
            allocator.__file__ = originals["allocator_file"]
            wrapper.__file__ = originals["wrapper_file"]
            allocator._state = originals["allocator_state"]
            allocator._volatile_counters = originals["allocator_volatile"]
            allocator._WAYPOINT_SIGNATURE_CACHE = originals["allocator_waypoint_signature"]
            id_allocator_0202.__file__ = originals["allocator_0202_file"]
            if hasattr(id_allocator_0202, "_STATE"):
                id_allocator_0202._STATE = originals["allocator_0202_state"]  # type: ignore[attr-defined]


def check_cold_reset_continues_from_disk(allocator: Any) -> None:
    with isolated_allocator(allocator, prefix="id_allocator_cold_") as store:
        _assert_equal(
            allocator.reserve_mission_plan_ids(3),
            [700_000_001, 700_000_002, 700_000_003],
            "initial missionPlanID reserve",
        )
        _assert_equal(
            allocator.reserve_imp_ids(2),
            [800_000_001, 800_000_002],
            "initial IMP reserve",
        )
        _assert_equal(
            allocator.reserve_path_ids(1, 2),
            [100_000_001, 100_000_002],
            "initial pathID reserve",
        )

        _reset_memory(allocator)
        _assert_equal(
            allocator.reserve_mission_plan_ids(2),
            [700_000_004, 700_000_005],
            "missionPlanID reserve after memory reset",
        )
        _assert_equal(
            allocator.reserve_imp_ids(1),
            [800_000_003],
            "IMP reserve after memory reset",
        )
        _assert_equal(
            allocator.reserve_path_id_blocks({1: 1, "2": 2}),
            {1: [100_000_003], 2: [200_000_001, 200_000_002]},
            "pathID block reserve after memory reset",
        )

        store.write_text(
            json.dumps(
                {
                    "missionPlanID": 700_000_010,
                    "individualMissionPackage": 800_000_010,
                    "individualMission": 900_000_010,
                    "pathID": {"1": 100_000_050, "2": 200_000_009},
                },
                separators=(",", ":"),
            ),
            encoding="utf-8",
        )
        _reset_memory(allocator)
        _assert_equal(
            allocator.reserve_mission_plan_ids(1),
            [700_000_011],
            "missionPlanID reserve after cold disk high-water",
        )
        _assert_equal(
            allocator.reserve_replan_id_bundle(
                path_count_by_aircraft={1: 2, 2: 1},
                imp_count=1,
                individual_mission_count=2,
            ),
            {
                "pathID": {
                    1: [100_000_051, 100_000_052],
                    2: [200_000_010],
                },
                "individualMissionPackage": [800_000_011],
                "individualMission": [900_000_011, 900_000_012],
            },
            "bundle reserve after cold disk high-water",
        )

        _assert_equal(allocator.reserve_waypoint_block(3), 50, "initial waypoint reserve")
        _reset_memory(allocator)
        _assert_equal(
            allocator.reserve_waypoint_block(1),
            50,
            "waypoint volatile cold-reset reserve",
        )
        stored = _stored_json(store)
        if "waypoint" in stored:
            fail(f"waypoint should remain volatile and absent from id_tracker: {stored!r}")
        if not store.with_name("id_tracker.json.lock").exists():
            fail("id allocator store lock file was not created")


def check_parallel_linear_reserve(allocator: Any) -> None:
    with isolated_allocator(allocator, prefix="id_allocator_linear_") as store:
        task_count = 12
        block_size = 3
        blocks = _run_parallel(task_count, lambda _idx: allocator.reserve_mission_plan_ids(block_size))
        flat = [int(value) for block in blocks for value in block]
        for idx, block in enumerate(blocks):
            _assert_contiguous_block(block, label=f"missionPlanID concurrent block {idx}")
        _assert_exact_range(
            flat,
            allocator.BASE["missionPlanID"],
            task_count * block_size,
            "missionPlanID concurrent reserve",
        )
        stored = _stored_json(store)
        _assert_equal(
            stored.get("missionPlanID"),
            allocator.BASE["missionPlanID"] + task_count * block_size - 1,
            "missionPlanID stored high-water after concurrent reserve",
        )


def check_parallel_path_blocks(allocator: Any) -> None:
    with isolated_allocator(allocator, prefix="id_allocator_path_") as store:
        task_count = 10
        blocks = _run_parallel(
            task_count,
            lambda _idx: allocator.reserve_path_id_blocks({2: 2, "3": 1}),
        )
        path2: list[int] = []
        path3: list[int] = []
        for idx, result in enumerate(blocks):
            _assert_contiguous_block(result[2], label=f"pathID aircraft 2 concurrent block {idx}")
            _assert_contiguous_block(result[3], label=f"pathID aircraft 3 concurrent block {idx}")
            path2.extend(int(value) for value in result[2])
            path3.extend(int(value) for value in result[3])
        _assert_exact_range(path2, allocator.BASE["pathID"][2], task_count * 2, "aircraft 2 pathID reserve")
        _assert_exact_range(path3, allocator.BASE["pathID"][3], task_count, "aircraft 3 pathID reserve")
        stored = _stored_json(store)
        stored_paths = stored.get("pathID") or {}
        _assert_equal(
            stored_paths.get("2"),
            allocator.BASE["pathID"][2] + task_count * 2 - 1,
            "aircraft 2 stored pathID high-water",
        )
        _assert_equal(
            stored_paths.get("3"),
            allocator.BASE["pathID"][3] + task_count - 1,
            "aircraft 3 stored pathID high-water",
        )


def check_parallel_bundle_reserve(allocator: Any) -> None:
    with isolated_allocator(allocator, prefix="id_allocator_bundle_") as store:
        task_count = 9
        bundles = _run_parallel(
            task_count,
            lambda _idx: allocator.reserve_replan_id_bundle(
                path_count_by_aircraft={4: 2},
                imp_count=1,
                individual_mission_count=2,
            ),
        )
        path4: list[int] = []
        imp_ids: list[int] = []
        individual_ids: list[int] = []
        for idx, bundle in enumerate(bundles):
            _assert_contiguous_block(bundle["pathID"][4], label=f"bundle pathID block {idx}")
            _assert_contiguous_block(bundle["individualMissionPackage"], label=f"bundle IMP block {idx}")
            _assert_contiguous_block(bundle["individualMission"], label=f"bundle individual block {idx}")
            path4.extend(int(value) for value in bundle["pathID"][4])
            imp_ids.extend(int(value) for value in bundle["individualMissionPackage"])
            individual_ids.extend(int(value) for value in bundle["individualMission"])
        _assert_exact_range(path4, allocator.BASE["pathID"][4], task_count * 2, "bundle pathID reserve")
        _assert_exact_range(imp_ids, allocator.BASE["individualMissionPackage"], task_count, "bundle IMP reserve")
        _assert_exact_range(
            individual_ids,
            allocator.BASE["individualMission"],
            task_count * 2,
            "bundle individual mission reserve",
        )
        stored = _stored_json(store)
        stored_paths = stored.get("pathID") or {}
        _assert_equal(
            stored_paths.get("4"),
            allocator.BASE["pathID"][4] + task_count * 2 - 1,
            "bundle stored pathID high-water",
        )
        _assert_equal(
            stored.get("individualMissionPackage"),
            allocator.BASE["individualMissionPackage"] + task_count - 1,
            "bundle stored IMP high-water",
        )
        _assert_equal(
            stored.get("individualMission"),
            allocator.BASE["individualMission"] + task_count * 2 - 1,
            "bundle stored individual high-water",
        )


def check_parallel_waypoint_blocks(allocator: Any) -> None:
    with isolated_allocator(allocator, prefix="id_allocator_waypoint_") as _store:
        task_count = 8
        block_size = 2
        starts = _run_parallel(task_count, lambda _idx: allocator.reserve_waypoint_block(block_size))
        values: list[int] = []
        for start in starts:
            values.extend(range(int(start), int(start) + block_size))
        _assert_exact_range(values, allocator.BASE["waypoint"], task_count * block_size, "waypoint concurrent reserve")

        _reset_memory(allocator)
        multi_blocks = _run_parallel(
            task_count,
            lambda _idx: allocator.reserve_waypoint_blocks([1, 2]),
        )
        values = []
        for idx, result in enumerate(multi_blocks):
            if len(result) != 2:
                fail(f"waypoint multi-block result length changed at {idx}: {result!r}")
            for block in result:
                start, end = block
                values.extend(range(int(start), int(end) + 1))
        _assert_exact_range(
            values,
            allocator.BASE["waypoint"],
            task_count * 3,
            "waypoint multi-block concurrent reserve",
        )


SUBPROCESS_WORKER = r"""
from __future__ import annotations

import importlib
import json
import os
import sys
from pathlib import Path

project_root = Path(sys.argv[1])
store = Path(sys.argv[2])
mode = sys.argv[3]

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("KU_ROLE", "mission")
os.environ.setdefault("REPLAN_ID_ALLOCATOR_TIMING", "0")
for path in reversed((
    project_root,
    project_root / "modules",
    project_root / "modules" / "mission_planning",
    project_root / "modules" / "mission_planning" / "MissionPlanner",
)):
    text = str(path)
    if path.exists() and text not in sys.path:
        sys.path.insert(0, text)

allocator = importlib.import_module("modules.mission_planning.engine.mission_generation.id_allocation.allocator")
allocator._STORE = store
allocator._state = {}
allocator._volatile_counters = {
    key: int(allocator.BASE[key]) - 1
    for key in getattr(allocator, "VOLATILE_KEYS", set())
}
allocator._WAYPOINT_SIGNATURE_CACHE = None
allocator._sync_active_db_scope = lambda: None
allocator._record_path_usage_many = lambda _updates: None
allocator._record_waypoint_usage = lambda _value, **_kwargs: None
allocator._refresh_waypoint_counter = lambda **_kwargs: None
allocator._emit_id_metric = lambda *_args, **_kwargs: None

if mode == "mission":
    result = allocator.reserve_mission_plan_ids(4)
elif mode == "bundle":
    result = allocator.reserve_replan_id_bundle(
        path_count_by_aircraft={5: 2},
        imp_count=1,
        individual_mission_count=1,
    )
else:
    raise SystemExit(f"unknown mode: {mode}")

print(json.dumps(result, ensure_ascii=False, sort_keys=True))
"""


def _run_subprocess_worker(store: Path, mode: str) -> Any:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            SUBPROCESS_WORKER,
            str(PROJECT_ROOT),
            str(store),
            str(mode),
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        fail(
            "subprocess allocator worker failed:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.stdout,
                result.stderr,
            )
        )
    try:
        return json.loads(result.stdout.strip())
    except Exception as exc:
        fail(f"subprocess allocator worker returned invalid JSON: {result.stdout!r} ({exc})")


def check_subprocess_file_lock_reserve(allocator: Any) -> None:
    with tempfile.TemporaryDirectory(prefix="id_allocator_process_") as tmp:
        store = Path(tmp) / "DSS_Internal" / "id_tracker.json"
        process_count = 8
        block_size = 4
        blocks = _run_parallel(process_count, lambda _idx: _run_subprocess_worker(store, "mission"))
        values = [int(value) for block in blocks for value in block]
        for idx, block in enumerate(blocks):
            _assert_contiguous_block([int(value) for value in block], label=f"process mission block {idx}")
        _assert_exact_range(
            values,
            allocator.BASE["missionPlanID"],
            process_count * block_size,
            "cross-process missionPlanID reserve",
        )
        stored = _stored_json(store)
        _assert_equal(
            stored.get("missionPlanID"),
            allocator.BASE["missionPlanID"] + process_count * block_size - 1,
            "cross-process missionPlanID stored high-water",
        )
        if not store.with_name("id_tracker.json.lock").exists():
            fail("cross-process reserve did not create id_tracker.json.lock")

    with tempfile.TemporaryDirectory(prefix="id_allocator_process_bundle_") as tmp:
        store = Path(tmp) / "DSS_Internal" / "id_tracker.json"
        process_count = 6
        bundles = _run_parallel(process_count, lambda _idx: _run_subprocess_worker(store, "bundle"))
        path5: list[int] = []
        imp_ids: list[int] = []
        individual_ids: list[int] = []
        for idx, bundle in enumerate(bundles):
            path_block = [int(value) for value in (bundle.get("pathID") or {}).get("5", [])]
            imp_block = [int(value) for value in bundle.get("individualMissionPackage") or []]
            individual_block = [int(value) for value in bundle.get("individualMission") or []]
            _assert_contiguous_block(path_block, label=f"process bundle path block {idx}")
            _assert_contiguous_block(imp_block, label=f"process bundle IMP block {idx}")
            _assert_contiguous_block(individual_block, label=f"process bundle individual block {idx}")
            path5.extend(path_block)
            imp_ids.extend(imp_block)
            individual_ids.extend(individual_block)
        _assert_exact_range(path5, allocator.BASE["pathID"][5], process_count * 2, "cross-process pathID bundle")
        _assert_exact_range(imp_ids, allocator.BASE["individualMissionPackage"], process_count, "cross-process IMP bundle")
        _assert_exact_range(individual_ids, allocator.BASE["individualMission"], process_count, "cross-process individual bundle")
        stored = _stored_json(store)
        stored_paths = stored.get("pathID") or {}
        _assert_equal(
            stored_paths.get("5"),
            allocator.BASE["pathID"][5] + process_count * 2 - 1,
            "cross-process bundle stored pathID high-water",
        )
        _assert_equal(
            stored.get("individualMissionPackage"),
            allocator.BASE["individualMissionPackage"] + process_count - 1,
            "cross-process bundle stored IMP high-water",
        )
        _assert_equal(
            stored.get("individualMission"),
            allocator.BASE["individualMission"] + process_count - 1,
            "cross-process bundle stored individual high-water",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke ID allocator cold-reset and concurrent reserve parity.")
    parser.parse_args()

    try:
        configure_import_paths()
        allocator = importlib.import_module(ALLOCATOR_MODULE)
        check_run_reset_contract(allocator)
        check_cold_reset_continues_from_disk(allocator)
        check_parallel_linear_reserve(allocator)
        check_parallel_path_blocks(allocator)
        check_parallel_bundle_reserve(allocator)
        check_parallel_waypoint_blocks(allocator)
        check_subprocess_file_lock_reserve(allocator)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("id allocator cold-reset/concurrent-reserve parity smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
