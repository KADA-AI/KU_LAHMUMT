from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


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


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"missing source file: {rel_path}")
    return path.read_bytes().decode("utf-8-sig", errors="ignore")


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    text = read_source(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing manual workflow owner-decision markers: {missing!r}")


def file_hash(rel_path: str) -> str:
    data = (PROJECT_ROOT / rel_path).read_bytes()
    return hashlib.sha256(data).hexdigest()


def check_decision_document() -> None:
    assert_source_contains(
        "docs/mission planning refactoring/67-manual-workflow-owner-decisions-progress.md",
        "| `manual/logic_test/division_test/**` | delete-hold | next-collab/planning-enhanced owner |",
        "| `manual/logic_test/dubins_test/**` | wrapper candidate | Dubins/flight-path owner |",
        "| `manual/MissionVisualizer/main_visualizer.py` | keep canonical | operator/manual visualization |",
        "| `MissionVisualizer` old import path | package-alias | operator/manual visualization |",
        "| `MissionPlanner/tools/main_visualizer.py` | wrapper | operator/manual visualization |",
        "| `MissionPlanner/portable_mission_bundle/**` | keep | portable/RL operator workflow |",
        "| `manual/lah_rl_planner_gui.py` | keep canonical | portable/RL operator workflow |",
        "| root `lah_rl_planner_gui.py` | wrapper | portable/RL compatibility launcher |",
        "| `MissionPlanner/tools/UAV_pattern/Nadir_BF/**` | keep active support | mission generation/runtime |",
        "| Other `MissionPlanner/tools/UAV_pattern/**` demos | archive-hold | manual prototype owner needed |",
        "No delete action is approved by this checkpoint.",
    )


def check_logic_test_decisions() -> None:
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/main.py",
        'os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")',
        "from division_planner_gui import main as gui_main",
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/division_test/division_planner_gui.py",
        "class DivisionPlannerWindow(QMainWindow):",
        'os.environ.get("DIVISION_TEST_FLOW_MODE", "initial")',
    )
    output_files = list((PROJECT_ROOT / "modules/mission_planning/manual/logic_test/division_test/output").rglob("*.json"))
    expect_equal("division_test golden/manual JSON count", len(output_files), 19)

    assert_source_contains(
        "modules/mission_planning/manual/logic_test/dubins_test/dubins_turn_link_gui.py",
        "def main() -> int:",
        "app = QApplication.instance() or QApplication(sys.argv)",
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/dubins_test/dubins_turn_link_logic.py",
        "def main(argv: Sequence[str] | None = None) -> int:",
        '"--path-policy"',
    )
    assert_source_contains(
        "modules/mission_planning/manual/logic_test/dubins_test/dubins_turn_link_gui.py",
        "def main() -> int:",
        "app = QApplication.instance() or QApplication(sys.argv)",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/dubins_turn_link.py",
        "def main(argv: Sequence[str] | None = None) -> int:",
        '"--path-policy"',
    )


def check_visualizer_and_tool_decisions() -> None:
    canonical = "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py"
    tool_wrapper = "modules/mission_planning/MissionPlanner/tools/main_visualizer.py"
    assert_source_contains(
        canonical,
        "class MissionPlanVisualizer(QWidget):",
        "def main() -> None:",
        "viewer = MissionPlanVisualizer()",
        "viewer.show()",
    )
    assert_source_contains(
        "modules/mission_planning/__init__.py",
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
        "class _CompatTargetModuleLoader",
    )
    assert_source_contains(
        tool_wrapper,
        "Compatibility wrapper for the canonical mission visualizer entrypoint.",
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
        "MissionPlanVisualizer = _MODULE.MissionPlanVisualizer",
        "main = _MODULE.main",
        '__all__ = ["MissionPlanVisualizer", "main"]',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/test_div_area.py",
        "tester = AreaSplitTester()",
        "plt.show()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/turn_link_visualizer.py",
        "_ = TurnLinkVisualizer()",
        "plt.show()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/DTA.py",
        "def main():",
        "plt.show()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/corridor_gui.py",
        "class CorridorGUI(QWidget):",
        "gui = CorridorGUI()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/corridor_planner.py",
        "if __name__ == \"__main__\":",
    )


def check_portable_and_rl_decisions() -> None:
    required = (
        "app.py",
        "run_portable.bat",
        "requirements.txt",
        "README.md",
        "portable_mission/web.py",
        "portable_mission/service.py",
        "portable_mission/env.py",
        "portable_mission/terrain.py",
        "portable_mission/hex_utils.py",
        "portable_mission/templates/index.html",
        "portable_mission/static/app.js",
        "portable_mission/static/app.css",
        "models/latest_model.zip",
        "models/model_config.json",
        "data/inputs/README.txt",
        "data/work/README.txt",
    )
    bundle = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/portable_mission_bundle"
    missing = [rel for rel in required if not (bundle / rel).exists()]
    if missing:
        fail(f"portable bundle owner-decision files missing: {missing!r}")

    assert_source_contains(
        "modules/mission_planning/manual/lah_rl_planner_gui.py",
        '_BUNDLE_ROOT = _MISSION_ROOT / "MissionPlanner" / "portable_mission_bundle"',
        '_MODEL_PATH = _BUNDLE_ROOT / "models" / "latest_model.zip"',
        "from portable_mission.env import PortableMissionEnv",
        "class LAHPlannerWindow(QMainWindow):",
    )
    assert_source_contains(
        "modules/mission_planning/lah_rl_planner_gui.py",
        'import_module("modules.mission_planning.manual.lah_rl_planner_gui")',
        "_MODULE.main()",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/service.py",
        'self.model_path = self.models_dir / "latest_model.zip"',
        'self.config_path = self.models_dir / "model_config.json"',
    )
    assert_source_contains(
        "modules/mission_planning/mission_planning_gui.py",
        "def _open_lah_rl_planner(self) -> None:",
        "from modules.mission_planning.manual.lah_rl_planner_gui import LAHPlannerWindow",
        "from lah_rl_planner_gui import LAHPlannerWindow",
    )


def check_uav_pattern_decisions() -> None:
    assert_source_contains(
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
        "from modules.mission_planning.MissionPlanner.tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import",
        "build_nadir_bf_overflight_coords",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/area_nadir_bf_planner.py",
        "def build_nadir_bf_overflight_coords(",
    )
    for rel_path in (
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_BF/visualization.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Interval_Round_Trip_BF/IRBF_DB_Generate.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_Sweep_ROI/main_example.py",
    ):
        expect_true(f"prototype/manual UAV pattern file exists: {rel_path}", (PROJECT_ROOT / rel_path).exists())


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke manual workflow owner decisions.")
    parser.parse_args()

    try:
        check_decision_document()
        check_logic_test_decisions()
        check_visualizer_and_tool_decisions()
        check_portable_and_rl_decisions()
        check_uav_pattern_decisions()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("manual workflow owner decision smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
