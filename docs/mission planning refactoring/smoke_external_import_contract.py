from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

MONITORED_PY_ROOTS = (
    PROJECT_ROOT / "modules" / "common",
    PROJECT_ROOT / "modules" / "monitoring",
    PROJECT_ROOT / "app",
)
MONITORED_PY_FILES = (PROJECT_ROOT / "run.py",)

EXPECTED_MISSION_PLANNING_IMPORTS = {
    (
        "app/ui/main_window.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("fov_db_path", "set_runtime_fov_db_path", "validate_fov_db_file"),
    ),
    (
        "modules/common/agent_status_snapshot.py",
        "modules.mission_planning.runtime.attack_tracking_state",
        ("update_from_agent_states",),
    ),
    (
        "modules/common/agent_status_snapshot.py",
        "modules.mission_planning.runtime.prior_tracking_state",
        ("update_from_agent_states",),
    ),
    (
        "modules/common/next_collab_replan_store.py",
        "modules.mission_planning.runtime.next_collab_replan_store",
        ("load_detail", "save_detail", "save_event"),
    ),
    (
        "modules/monitoring/gui/tabs/monitoring_visualization_tab.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("get_runtime_str",),
    ),
    (
        "modules/monitoring/gui/tabs/monitoring_visualization_tab.py",
        "modules.mission_planning.pipelines.mission_path_trim",
        ("DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS",),
    ),
    (
        "modules/monitoring/logic/init_replan.py",
        "modules.mission_planning.MissionPlanner.data_def.id_allocator",
        ("next_mission_plan_id", "reserve_mission_plan_ids"),
    ),
    (
        "modules/monitoring/logic/mission_update.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("load_fov_db_rows",),
    ),
    (
        "modules/monitoring/logic/next_collab_replan.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("get_runtime_float", "get_runtime_str"),
    ),
    (
        "modules/monitoring/logic/next_collab_replan.py",
        "modules.mission_planning.runtime",
        ("next_collab_replan_runtime", "next_collab_replan_store"),
    ),
    (
        "modules/monitoring/logic/prior_mission_replan.py",
        "modules.mission_planning.runtime.attack_tracking_state",
        ("resolve_plan_lineage_ids",),
    ),
    (
        "modules/monitoring/logic/prior_mission_replan.py",
        "modules.mission_planning.runtime.prior_tracking_state",
        ("list_active_prior_assignments", "mark_prior_handoff"),
    ),
    (
        "modules/monitoring/logic/target_detection_replan.py",
        "modules.mission_planning.runtime.attack_assignment_state",
        (
            "clear_deferred_attack_targets",
            "defer_attack_targets",
            "get_used_manned_ids",
            "list_deferred_attack_targets",
        ),
    ),
    (
        "modules/monitoring/logic/target_detection_replan.py",
        "modules.mission_planning.runtime.attack_tracking_state",
        ("list_active_tracking_assignments", "resolve_plan_lineage_ids"),
    ),
    (
        "modules/monitoring/logic/turn_radius_monitor.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("get_runtime_float",),
    ),
    (
        "modules/monitoring/monitoring_gui.py",
        "modules.mission_planning.MissionPlanner.runtime_settings",
        ("get_runtime_float",),
    ),
    (
        "modules/monitoring/monitoring_gui.py",
        "modules.mission_planning.runtime.attack_assignment_state",
        (
            "clear_pending_manned_assignments",
            "commit_pending_manned_assignment",
            "commit_pending_manned_assignments",
        ),
    ),
    (
        "run.py",
        "modules.mission_planning.MissionPlanner.data_def",
        ("id_allocator",),
    ),
    (
        "run.py",
        "modules.mission_planning.MissionPlanner.data_def",
        ("id_allocator_0202",),
    ),
}


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"missing source file: {rel_path}")
    return path.read_bytes().decode("utf-8-sig", errors="ignore")


def parse_source(path: Path) -> ast.Module:
    return ast.parse(path.read_bytes().decode("utf-8-sig", errors="ignore"), filename=str(path))


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    text = read_source(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing external import contract markers: {missing!r}")


def collect_external_mission_imports() -> set[tuple[str, str, tuple[str, ...]]]:
    paths: list[Path] = []
    for root in MONITORED_PY_ROOTS:
        paths.extend(sorted(root.rglob("*.py")))
    paths.extend(MONITORED_PY_FILES)

    imports: set[tuple[str, str, tuple[str, ...]]] = set()
    for path in paths:
        tree = parse_source(path)
        rel_path = path.relative_to(PROJECT_ROOT).as_posix()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                module_name = node.module or ""
                if not module_name.startswith("modules.mission_planning"):
                    continue
                imports.add((rel_path, module_name, tuple(alias.name for alias in node.names)))
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name.startswith("modules.mission_planning"):
                        imports.add((rel_path, alias.name, ()))
    return imports


def check_external_import_inventory() -> None:
    actual = collect_external_mission_imports()
    if actual != EXPECTED_MISSION_PLANNING_IMPORTS:
        missing = sorted(EXPECTED_MISSION_PLANNING_IMPORTS - actual)
        extra = sorted(actual - EXPECTED_MISSION_PLANNING_IMPORTS)
        fail(f"external mission_planning import inventory changed: missing={missing!r}, extra={extra!r}")


def check_launcher_string_contracts() -> None:
    assert_source_contains(
        "run.py",
        'data_def_dir = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "data_def"',
        'from modules.mission_planning.MissionPlanner.data_def import id_allocator',
        'from modules.mission_planning.MissionPlanner.data_def import id_allocator_0202',
        "base = id_allocator.BASE",
        "store = Path(id_allocator.__file__).resolve().parent / \"id_tracker.json\"",
        "id_allocator._state.clear()",
        "id_allocator._volatile_counters = {",
        "id_allocator.VOLATILE_KEYS",
        'base_0202 = getattr(id_allocator_0202, "_BASE_STATE", {"individualMissionID": 0})',
        'if hasattr(id_allocator_0202, "_STATE"):',
        'store_0202 = Path(id_allocator_0202.__file__).resolve().parent / "id_tracker_0202.json"',
        'counter_file = Path(id_allocator.__file__).resolve().parent / "_id_counters.json"',
        '"mission": "mission_planning_gui.py"',
        'if r in ("assignment", "mission", "mission_planning", "mmr"):',
        '"mission_planning_gui.py"',
    )
    assert_source_contains(
        "app/ui/main_window.py",
        "from modules.mission_planning.MissionPlanner.runtime_settings import",
        'root / "modules" / "mission_planning" / "mission_planning_gui.py"',
        'root / "app"     / "modules" / "mission_planning" / "mission_planning_gui.py"',
        'root / "modules" / "decision_support" / "assignment_planning_gui.py"',
        "set_runtime_fov_db_path",
        "validate_fov_db_file",
    )
    assert_source_contains(
        "modules/common/button_wiring.py",
        '"assignment": "mission_planning_gui.py"',
        "orch._launch_gui(sn)",
    )
    assert_source_contains(
        "modules/common/ops_checklist.py",
        '"mission_planning_gui.py": "assignment"',
        '"mission_planning":        "assignment"',
        'if s in ("assignment", "mission", "mmr", "mission_planning")',
    )


def check_runtime_state_import_fallbacks() -> None:
    assert_source_contains(
        "modules/common/agent_status_snapshot.py",
        "try:",
        "from modules.mission_planning.runtime.attack_tracking_state import",
        "_update_attack_tracking_state = None",
        "from modules.mission_planning.runtime.prior_tracking_state import",
        "_update_prior_tracking_state = None",
    )
    assert_source_contains(
        "modules/monitoring/logic/target_detection_replan.py",
        "from modules.mission_planning.runtime.attack_assignment_state import",
        "def get_used_manned_ids(_input_package_id: int | None) -> set[int]:",
        "from modules.mission_planning.runtime.attack_tracking_state import",
        "def list_active_tracking_assignments() -> list[dict[str, Any]]:",
    )
    assert_source_contains(
        "modules/monitoring/monitoring_gui.py",
        "from modules.mission_planning.runtime.attack_assignment_state import",
        "def clear_pending_manned_assignments(_mission_plan_ids: object) -> list[int]:",
        "def commit_pending_manned_assignment(_mission_plan_id: int | None) -> int | None:",
    )


def check_next_collab_and_runtime_settings_contracts() -> None:
    assert_source_contains(
        "modules/common/next_collab_replan_store.py",
        "Compatibility wrapper for the active mission-planning next-collab store.",
        "from modules.mission_planning.runtime.next_collab_replan_store import",
        '__all__ = ["load_detail", "save_detail", "save_event"]',
    )
    assert_source_contains(
        "modules/monitoring/logic/next_collab_replan.py",
        "from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float, get_runtime_str",
        "from modules.mission_planning.runtime import next_collab_replan_runtime, next_collab_replan_store",
        "REPLAN_REASON = next_collab_replan_runtime.REPLAN_REASON",
    )
    assert_source_contains(
        "modules/monitoring/gui/tabs/monitoring_visualization_tab.py",
        "from modules.common import db_paths, next_collab_replan_store",
        "from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_str",
        "from modules.mission_planning.pipelines.mission_path_trim import DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS",
    )
    assert_source_contains(
        "modules/monitoring/logic/init_replan.py",
        "from modules.mission_planning.MissionPlanner.data_def.id_allocator import",
        "next_mission_plan_id",
        "reserve_mission_plan_ids",
    )
    assert_source_contains(
        "modules/monitoring/logic/mission_update.py",
        "from modules.mission_planning.MissionPlanner.runtime_settings import load_fov_db_rows",
    )
    assert_source_contains(
        "modules/monitoring/logic/turn_radius_monitor.py",
        "from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke monitoring/common/app external mission_planning imports.")
    parser.parse_args()

    try:
        check_external_import_inventory()
        check_launcher_string_contracts()
        check_runtime_state_import_fallbacks()
        check_next_collab_and_runtime_settings_contracts()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("monitoring/common/app external import contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
