from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    path = PROJECT_ROOT / rel_path
    text = path.read_text(encoding="utf-8", errors="ignore")
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing manual planner flow-mode markers: {missing!r}")


def check_next_area_flow_mode() -> None:
    assert_source_contains(
        "modules/mission_planning/next_area_mode/config.py",
        'FLOW_MODE_ENV_KEY = "MISSION_NEXT_AREA_FLOW_MODE"',
        'MISSION_AREA = "area"',
        'MISSION_LINE = "line"',
    )
    assert_source_contains(
        "modules/mission_planning/next_area_mode/main.py",
        "from modules.mission_planning.next_area_mode.config import FLOW_MODE_ENV_KEY",
        'os.environ.setdefault(FLOW_MODE_ENV_KEY, "initial")',
        "from modules.mission_planning.next_area_mode.planner_window import main as gui_main",
    )
    assert_source_contains(
        "modules/mission_planning/next_area_mode/tab.py",
        'self.cmb_flow.addItem("Initial", "initial")',
        'self.cmb_flow.addItem("Replan", "replan")',
        'os.environ.get(FLOW_MODE_ENV_KEY, "initial")',
        'os.environ[FLOW_MODE_ENV_KEY] = "replan" if flow_value == "replan" else "initial"',
    )
    assert_source_contains(
        "modules/mission_planning/next_area_mode/planner_window.py",
        "def _flow_mode(self) -> str:",
        'os.environ.get(FLOW_MODE_ENV_KEY, "initial")',
        'return "replan" if raw == "replan" else "initial"',
        "def _is_replan_flow(self) -> bool:",
        'return self._flow_mode() == "replan"',
        "use_replan_flow = self._is_replan_flow()",
        "apply_assignment=use_replan_flow",
        'stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]',
    )
    assert_source_contains(
        "modules/mission_planning/runtime/next_collab_line_runner.py",
        "from modules.mission_planning.next_area_mode.config import",
        "from modules.mission_planning.next_area_mode.planner_window import",
        "class _HeadlessLinePlanner:",
        "planner.state.mode = MODE_MISSION_READY",
    )


def check_next_collab_division_flow_mode() -> None:
    assert_source_contains(
        "modules/mission_planning/planners/next_collab_division/main.py",
        "def _find_project_root(start: Path) -> Path:",
        "def _prepare_path() -> None:",
        'os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")',
        "from modules.mission_planning.planners.next_collab_division.division_planner_gui import main as gui_main",
    )
    assert_source_contains(
        "modules/mission_planning/planners/next_collab_division/division_planner_gui.py",
        "from ._planner_window import DivisionPlannerWindow",
        "app = QApplication.instance() or QApplication(sys.argv)",
        "win = DivisionPlannerWindow()",
    )
    assert_source_contains(
        "modules/mission_planning/planners/next_collab_division/_planner_window.py",
        "def _flow_mode(self) -> str:",
        'os.environ.get("DIVISION_TEST_FLOW_MODE", "initial")',
        'return "replan" if raw == "replan" else "initial"',
        "def _is_replan_flow(self) -> bool:",
        'return self._flow_mode() == "replan"',
        "use_replan_flow = self._is_replan_flow()",
        "apply_assignment=use_replan_flow",
        'stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]',
        "def _mid_line_no_split_mode(",
    )
    assert_source_contains(
        "modules/mission_planning/runtime/next_collab_division_runner.py",
        "from modules.mission_planning.planners.next_collab_division._planner_window import",
        "class _HeadlessDivisionPlanner:",
        "planner.state.mode = MODE_MISSION_READY",
        "planner._mid_line_no_split_mode()",
    )


def check_logic_test_division_mirror_flow_mode() -> None:
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/main.py",
        "def _prepare_path() -> None:",
        'os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")',
        "from division_planner_gui import main as gui_main",
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/division_planner_gui.py",
        "def _flow_mode(self) -> str:",
        'os.environ.get("DIVISION_TEST_FLOW_MODE", "initial")',
        'return "replan" if raw == "replan" else "initial"',
        "def _is_replan_flow(self) -> bool:",
        'return self._flow_mode() == "replan"',
        "use_replan_flow = self._is_replan_flow()",
        "apply_assignment=use_replan_flow",
        'stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]',
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke next-collab/next-area manual planner flow modes.")
    parser.parse_args()

    try:
        check_next_area_flow_mode()
        check_next_collab_division_flow_mode()
        check_logic_test_division_mirror_flow_mode()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("manual planner flow-mode smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
