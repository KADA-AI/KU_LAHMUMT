from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"missing entrypoint file: {rel_path}")
    return path.read_text(encoding="utf-8", errors="ignore")


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    text = read_source(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing manual/operator entrypoint markers: {missing!r}")


def check_public_operator_launchers() -> None:
    assert_source_contains(
        "modules/mission_planning/mission_planning_gui.py",
        "configure_mission_role()  # MMR",
        "def _smoke_launch_main() -> int:",
        "from modules.mission_planning.app.gui_entrypoint import smoke_launch_main",
        "return smoke_launch_main(sys.modules[__name__])",
        "from modules.mission_planning.app.gui_entrypoint import run_public_gui_entrypoint",
        "sys.exit(run_public_gui_entrypoint(sys.modules[__name__]))",
        "def _open_lah_rl_planner(self) -> None:",
        "from modules.mission_planning.manual.lah_rl_planner_gui import LAHPlannerWindow",
        "from lah_rl_planner_gui import LAHPlannerWindow",
        "self._lah_rl_win = LAHPlannerWindow(self)",
    )
    assert_source_contains(
        "modules/mission_planning/app/gui_entrypoint.py",
        'SMOKE_ENV_KEY = "MISSION_PLANNING_GUI_SMOKE_LAUNCH"',
        "def smoke_launch_main(gui_module: ModuleType | Any) -> int:",
        "app = gui_module.QApplication([sys.argv[0]])",
        "gui_module.load_shared_stylesheet(app, gui_module.PROJECT_ROOT)",
        "def run_gui(gui_module: ModuleType | Any, argv: Sequence[str] | None = None) -> int:",
        "win = gui_module.MainWindow()",
        "gui_module.apply_initial_visibility(app, win, gui_module.position_window_from_env)",
        "def run_public_gui_entrypoint(",
    )
    assert_source_contains(
        "modules/mission_planning/manual/lah_rl_planner_gui.py",
        "_MISSION_ROOT = _FILE_DIR.parent",
        '_BUNDLE_ROOT = _MISSION_ROOT / "MissionPlanner" / "portable_mission_bundle"',
        '_MODEL_PATH = _BUNDLE_ROOT / "models" / "latest_model.zip"',
        'if str(_BUNDLE_ROOT) not in sys.path:',
        "sys.modules[_PM_PKG] = _pkg",
        "from portable_mission.hex_utils import",
        "from portable_mission.env import PortableMissionEnv",
        "def main() -> None:",
        "app = QApplication(sys.argv)",
        "win = LAHPlannerWindow()",
        "win.show()",
    )
    assert_source_contains(
        "modules/mission_planning/lah_rl_planner_gui.py",
        "Backward-compatible wrapper for the manual LAH RL planner GUI.",
        'import_module("modules.mission_planning.manual.lah_rl_planner_gui")',
        'if __name__ == "__main__":',
        "_MODULE.main()",
    )


def check_manual_visualizer_entrypoints() -> None:
    for rel_path in (
        "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py",
        "modules/mission_planning/legacy/apps/MissionVisualizer/main_visualizer.py",
        "modules/mission_planning/legacy/MissionPlanner_tools/main_visualizer.py",
    ):
        assert_source_contains(
            rel_path,
            "def main() -> None:",
            "app = QApplication.instance() or QApplication(sys.argv)",
            "viewer = MissionPlanVisualizer()",
            "viewer.show()",
            "if __name__ == \"__main__\":",
            "main()",
        )
    assert_source_contains(
        "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py",
        "from modules.mission_planning._paths import project_root",
        "PROJECT_ROOT = project_root()",
    )
    assert_source_contains(
        "modules/mission_planning/__init__.py",
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
        "class _CompatTargetModuleLoader",
        "def _resolve_compat_package_alias(",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/main_visualizer.py",
        "Compatibility wrapper for the canonical mission visualizer entrypoint.",
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
        "MissionPlanVisualizer = _MODULE.MissionPlanVisualizer",
        "main = _MODULE.main",
        '__all__ = ["MissionPlanVisualizer", "main"]',
        "if __name__ == \"__main__\":",
        "main()",
    )
    assert_source_contains(
        "modules/mission_planning/legacy/compat_packages/MissionVisualizer/main_visualizer.py",
        "Backward-compatible wrapper for the MissionVisualizer entry point.",
        "from modules.mission_planning.legacy.apps.MissionVisualizer.main_visualizer import MissionPlanVisualizer, main",
        '__all__ = ["MissionPlanVisualizer", "main"]',
        "if __name__ == \"__main__\":",
        "main()",
    )


def check_next_area_and_division_entrypoints() -> None:
    assert_source_contains(
        "modules/mission_planning/next_area_mode/main.py",
        "def _prepare_path() -> None:",
        "multiprocessing.freeze_support()",
        "from modules.mission_planning.next_area_mode.config import FLOW_MODE_ENV_KEY",
        'os.environ.setdefault(FLOW_MODE_ENV_KEY, "initial")',
        "from modules.mission_planning.next_area_mode.planner_window import main as gui_main",
        "raise SystemExit(main())",
    )
    assert_source_contains(
        "modules/mission_planning/next_area_mode/config.py",
        'FLOW_MODE_ENV_KEY = "MISSION_NEXT_AREA_FLOW_MODE"',
    )
    assert_source_contains(
        "modules/mission_planning/next_area_mode/planner_window.py",
        "class NextAreaPlanningWindow(QMainWindow):",
        'os.environ.get(FLOW_MODE_ENV_KEY, "initial")',
        "app = QApplication.instance() or QApplication(sys.argv)",
        "win = NextAreaPlanningWindow()",
    )
    assert_source_contains(
        "modules/mission_planning/planners/next_collab_division/main.py",
        "def _find_project_root(start: Path) -> Path:",
        "def _prepare_path() -> None:",
        "multiprocessing.freeze_support()",
        'os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")',
        "from modules.mission_planning.planners.next_collab_division.division_planner_gui import main as gui_main",
        "raise SystemExit(main())",
    )
    assert_source_contains(
        "modules/mission_planning/planners/next_collab_division/division_planner_gui.py",
        "app = QApplication.instance() or QApplication(sys.argv)",
        "win = DivisionPlannerWindow()",
        "win.show()",
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/main.py",
        "def _prepare_path() -> None:",
        "multiprocessing.freeze_support()",
        'os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")',
        "from division_planner_gui import main as gui_main",
        "raise SystemExit(main())",
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/division_planner_gui.py",
        "class DivisionPlannerWindow(QMainWindow):",
        'os.environ.get("DIVISION_TEST_FLOW_MODE", "initial")',
        "app = QApplication.instance() or QApplication(sys.argv)",
        "win = DivisionPlannerWindow()",
    )
    assert_source_contains(
        "modules/mission_planning/legacy/compat_packages/next_area_mode/main.py",
        "Backward-compatible wrapper for next-area-mode entry point.",
        "from modules.mission_planning.legacy.apps.next_area_mode.main import main",
        "raise SystemExit(main())",
    )


def check_auxiliary_tool_entrypoints() -> None:
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/main_MP.py",
        "def _prepare_path() -> None:",
        "from PyQt5.QtWidgets import QApplication",
        "from planning_enhanced.gui import MainWindow",
        "app = QApplication.instance() or QApplication(sys.argv)",
        "win = MainWindow()",
        "raise SystemExit(main())",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/corridor_gui.py",
        "class CorridorGUI(QWidget):",
        "app = QApplication(sys.argv)",
        "gui = CorridorGUI()",
        "gui.show()",
        "sys.exit(app.exec_())",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/corridor_planner.py",
        "if __name__ == \"__main__\":",
    )
    for rel_path, class_marker in (
        ("modules/mission_planning/MissionPlanner/tools/test_div_area.py", "tester = AreaSplitTester()"),
        ("modules/mission_planning/MissionPlanner/tools/turn_link_visualizer.py", "_ = TurnLinkVisualizer()"),
        ("modules/mission_planning/MissionPlanner/tools/DTA.py", "def main():"),
    ):
        assert_source_contains(rel_path, "if __name__ == \"__main__\":", class_marker)
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/lah_attack_assistance.py",
        "parser = argparse.ArgumentParser(",
        'parser.add_argument("--friendly-lat"',
        'parser.add_argument("--enemy-lat"',
        'parser.add_argument("--output-json"',
        "if __name__ == \"__main__\":",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/dubins_turn_link.py",
        'argparse.ArgumentParser(description="Dubins turn-link test utility for two input line segments.")',
        'parser.add_argument("--prev-start"',
        'parser.add_argument("--next-end"',
        '"--path-policy"',
        "def main(argv: Sequence[str] | None = None) -> int:",
        "print(format_result(result))",
        "raise SystemExit(main())",
    )
    assert_source_contains(
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
        "from modules.mission_planning.MissionPlanner.data_def.dubins_turn_link import",
    )


def check_portable_bundle_entrypoints() -> None:
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/app.py",
        "ROOT = Path(__file__).resolve().parent",
        "from portable_mission import create_app",
        "app = create_app(ROOT)",
        'host = os.environ.get("MISSION_APP_HOST", "127.0.0.1")',
        'port = int(os.environ.get("MISSION_APP_PORT", "8877"))',
        "app.run(host=host, port=port, debug=False)",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/run_portable.bat",
        '@echo off',
        'cd /d "%~dp0"',
        'python app.py',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/README.md",
        "python app.py",
        "run_portable.bat",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/web.py",
        "def create_app(bundle_root: Path | None = None) -> Flask:",
        "service = MissionService(root)",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/service.py",
        'self.model_path = self.models_dir / "latest_model.zip"',
        'self.config_path = self.models_dir / "model_config.json"',
    )


def check_inventory_docs_cover_scope() -> None:
    assert_source_contains(
        "docs/mission planning refactoring/04-deletion-candidates.md",
        "`mission_planning_gui.py`",
        "`manual/MissionVisualizer/main_visualizer.py`",
        "`MissionPlanner/tools/main_visualizer.py`",
        "`MissionPlanner/portable_mission_bundle/app.py`, `requirements.txt`, `run_portable.bat`",
    )
    assert_source_contains(
        "docs/mission planning refactoring/07-third-review-risk-audit.md",
        "manual/tool entrypoint inventory",
        "duplicate visualizer public entrypoint decision",
        "portable bundle `python app.py`/`run_portable.bat` smoke with env port override",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/FFAR_list_class.py",
        "far = FAR(polygon, start, goal)",
        "plt.show()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_BF/visualization.py",
        "import matplotlib.pyplot as plt",
        "plt.show()",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke manual/operator entrypoint inventory.")
    parser.parse_args()

    try:
        check_public_operator_launchers()
        check_manual_visualizer_entrypoints()
        check_next_area_and_division_entrypoints()
        check_auxiliary_tool_entrypoints()
        check_portable_bundle_entrypoints()
        check_inventory_docs_cover_scope()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("manual/operator entrypoint inventory smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
