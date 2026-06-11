from __future__ import annotations

import argparse
import ast
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
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_source(rel_path: str) -> ast.Module:
    return ast.parse(read_source(rel_path), filename=rel_path)


def assert_source_contains(rel_path: str, *snippets: str) -> str:
    text = read_source(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing HTML/PNG artifact source markers: {missing!r}")
    return text


def function_arg_names(rel_path: str, function_name: str) -> list[str]:
    for node in ast.walk(parse_source(rel_path)):
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            return [arg.arg for arg in node.args.args]
    fail(f"{rel_path} missing function {function_name}")
    return []


def class_init_arg_names(rel_path: str, class_name: str) -> list[str]:
    for node in ast.walk(parse_source(rel_path)):
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for child in node.body:
            if isinstance(child, ast.FunctionDef) and child.name == "__init__":
                return [arg.arg for arg in child.args.args] + [
                    arg.arg for arg in child.args.kwonlyargs
                ]
    fail(f"{rel_path} missing {class_name}.__init__")
    return []


def check_active_mission_visualization_tab() -> None:
    rel_path = "modules/mission_planning/app/visualization/mission_visualization_tab.py"
    expect_equal("_default_map_html_path args", function_arg_names(rel_path, "_default_map_html_path"), [])
    expect_true(
        "MissionVisualizationTab keeps map_html_path injection",
        "map_html_path" in class_init_arg_names(rel_path, "MissionVisualizationTab"),
    )
    assert_source_contains(
        rel_path,
        "project_root = Path(__file__).resolve().parents[4]",
        'temp_dir = project_root / "temp"',
        'return temp_dir / "mission_planning_map.html"',
        'dir_mp = db_root / "MissionPlan"',
        'dir_imp = db_root / "IndividualMissionPlan"',
        'dir_fp = db_root / "FlightPath"',
        'entry.get("individualMissionPackageID") or entry.get("individualMissionPlanPackageID")',
        'fp_json.get("waypointList") or fp_json.get("lahWaypointList") or []',
        "fmap.save(str(self._map_html_path))",
        "QUrl.fromLocalFile(str(self._map_html_path))",
    )


def check_corridor_gui_map_html() -> None:
    rel_path = "modules/mission_planning/MissionPlanner/corridor_gui.py"
    expect_equal("_map_html_path args", function_arg_names(rel_path, "_map_html_path"), [])
    assert_source_contains(
        rel_path,
        '_PROJECT_ROOT = Path(__file__).resolve().parents[3]',
        '_TEMP_DIR = _PROJECT_ROOT / "temp"',
        'return _TEMP_DIR / "map.html"',
        "class MapBridge(QObject):",
        "pointClicked = pyqtSignal(float, float)",
        "fmap = folium.Map(location=center, zoom_start=12)",
        "fmap.save(str(self._map_html_path))",
        'qrc:///qtwebchannel/qwebchannel.js',
        'html.replace("</body>", js + "</body>")',
        "self.map_view.setUrl(QUrl.fromLocalFile(str(self._map_html_path.resolve())))",
    )


def check_manual_visualizer_map_html() -> None:
    for rel_path in (
        "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py",
        "modules/mission_planning/legacy/apps/MissionVisualizer/main_visualizer.py",
        "modules/mission_planning/legacy/MissionPlanner_tools/main_visualizer.py",
    ):
        assert_source_contains(
            rel_path,
            'Path(tempfile.mkdtemp(prefix="mission_visualizer_"))',
            'self._map_html = self._map_dir / "mission_map.html"',
            "fmap.save(self._map_html)",
            'html = self._map_html.read_text(encoding="utf-8")',
            "base_url = QUrl.fromLocalFile(str(self._map_dir))",
            "self.map_view.setHtml(html, base_url)",
        )
    assert_source_contains(
        "modules/mission_planning/__init__.py",
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
        "class _CompatTargetModuleLoader",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/tools/main_visualizer.py",
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
        "MissionPlanVisualizer = _MODULE.MissionPlanVisualizer",
        "main = _MODULE.main",
    )


def check_enhanced_map_html_renderer() -> None:
    rel_path = "modules/mission_planning/MissionPlanner/planning_enhanced/map/map_renderer.py"
    expect_equal(
        "build_map_html args",
        function_arg_names(rel_path, "build_map_html"),
        ["cmpk", "mrpk", "split_result"],
    )
    assert_source_contains(
        rel_path,
        "def build_map_html(",
        'tiles="OpenStreetMap"',
        "_add_0203_layers(m, mrpk)",
        "_add_original_missions(m, cmpk)",
        "_add_split_result_layers(m, split_result)",
        "_add_legend(m, split_result)",
        "return m.get_root().render()",
        "Mission Split Tester",
        "Original: mission order color-coded",
    )


def check_attack_visualization_png_contract() -> None:
    rel_path = "modules/mission_planning/MissionPlanner/data_def/lah_attack_assistance.py"
    expect_equal(
        "save_attack_visualization args",
        function_arg_names(rel_path, "save_attack_visualization"),
        [
            "output_path",
            "elevation",
            "geotransform",
            "polygons",
            "friendly_world",
            "enemy_world",
            "best",
            "radius_m",
            "raster_paths",
            "used_rasters",
        ],
    )
    assert_source_contains(
        rel_path,
        'matplotlib.use("Agg")',
        "output_path = os.path.abspath(output_path)",
        "if os.path.isdir(output_path):",
        'output_path = os.path.join(output_path, "attack_visualization.png")',
        "elif not os.path.splitext(output_path)[1]:",
        'output_path = f"{output_path}.png"',
        'os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)',
        'fig.savefig(output_path, dpi=180, bbox_inches="tight")',
        "return output_path",
        'parser.add_argument("--save-png"',
        'parser.add_argument("--output-json"',
        "if args.save_png:",
        "saved_png = save_attack_visualization(",
        'result["visualization_png"] = saved_png',
        "Visualization PNG :",
    )


def check_attack_helper_subprocess_boundary() -> None:
    assert_source_contains(
        "modules/mission_planning/pipelines/mission_planning_attack_helpers.py",
        '"lah_attack_assistance.py"',
        '"--friendly-lat"',
        '"--friendly-lon"',
        '"--enemy-lat"',
        '"--enemy-lon"',
        '"--output-json"',
        "subprocess.run(cmd, capture_output=True, text=True, check=False)",
    )
    assert_source_contains(
        "modules/mission_planning/replanning/triggers/attack/pipeline.py",
        "def _compute_attack_point_subprocess(",
        '"lah_attack_assistance.py"',
        '"--output-json"',
        "subprocess.run(",
        "def _compute_attack_point(",
    )


def check_static_output_artifact_inventory() -> None:
    assert_source_contains(
        "docs/mission planning refactoring/04-deletion-candidates.md",
        "`mission_planning_map.html`, `map.html`, `mission_map.html`",
        "`attack_visualization.png` and attack assistance PNG outputs",
    )
    for rel_path in (
        "modules/mission_planning/manual/reference/map.html",
        "modules/mission_planning/MissionPlanner/map.html",
        "modules/mission_planning/legacy/static/map.html",
    ):
        expect_true(f"static generated map artifact exists: {rel_path}", (PROJECT_ROOT / rel_path).exists())


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke HTML/PNG output artifact manifest.")
    parser.parse_args()

    try:
        check_active_mission_visualization_tab()
        check_corridor_gui_map_html()
        check_manual_visualizer_map_html()
        check_enhanced_map_html_renderer()
        check_attack_visualization_png_contract()
        check_attack_helper_subprocess_boundary()
        check_static_output_artifact_inventory()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("HTML/PNG output artifact manifest smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
