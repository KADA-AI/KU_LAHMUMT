from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MISSION_DIR = PROJECT_ROOT / "modules" / "mission_planning"
MISSION_PLANNER_DIR = MISSION_DIR / "MissionPlanner"
GUI_SCRIPT = MISSION_DIR / "mission_planning_gui.py"
LAUNCH_ENV_PARITY_SMOKE = PROJECT_ROOT / "docs" / "mission planning refactoring" / "smoke_launch_env_parity.py"
FORBIDDEN_PROJECT_ROOT_BARE_SHIMS = (
    PROJECT_ROOT / "AnS",
    PROJECT_ROOT / "data_def",
    PROJECT_ROOT / "config.py",
)


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def base_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    env.setdefault("KU_SHOW_RUN_CONSOLE", "1")
    env.setdefault("KU_SHOW_MODULE_CONSOLES", "0")
    return env


def run_python_case(label: str, cwd: Path, code: str, *, timeout_s: float = 30.0) -> str:
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=cwd,
        env=base_env(),
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if result.returncode != 0:
        fail(
            "{} failed with code {}\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                label,
                result.returncode,
                result.stdout,
                result.stderr,
            )
        )
    return result.stdout


def check_project_root_bare_shims_absent() -> None:
    existing = [path for path in FORBIDDEN_PROJECT_ROOT_BARE_SHIMS if path.exists()]
    if existing:
        fail("project-root bare import shims must stay absent: " + ", ".join(str(path) for path in existing))


def check_repo_root_bootstrapped_bare_imports() -> None:
    code = (
        "import importlib\n"
        "from pathlib import Path\n"
        "project_root = Path.cwd()\n"
        "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
        "runtime.ensure_mission_planner_import_paths(project_root)\n"
        "ans = importlib.import_module('AnS')\n"
        "ans_pipeline = importlib.import_module('AnS.mission_pipeline')\n"
        "data_def = importlib.import_module('data_def')\n"
        "d0302 = importlib.import_module('data_def.d0302')\n"
        "d0303 = importlib.import_module('data_def.d0303')\n"
        "d0304 = importlib.import_module('data_def.d0304')\n"
        "id_allocator = importlib.import_module('data_def.id_allocator')\n"
        "config = importlib.import_module('config')\n"
        "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
        "canonical_pipeline = importlib.import_module('modules.mission_planning.MissionPlanner.AnS.mission_pipeline')\n"
        "canonical_config = importlib.import_module('modules.mission_planning.MissionPlanner.config')\n"
        "if ans is not canonical_ans:\n"
        "    raise SystemExit('repo root bootstrapped AnS identity split')\n"
        "if ans_pipeline is not canonical_pipeline:\n"
        "    raise SystemExit('repo root bootstrapped AnS.mission_pipeline identity split')\n"
        "if not callable(ans.run_divide_and_pattern):\n"
        "    raise SystemExit('repo root bootstrapped AnS run_divide_and_pattern missing')\n"
        "if not callable(data_def.build_lah_flight_plans_fixed):\n"
        "    raise SystemExit('repo root bootstrapped data_def package export missing')\n"
        "if not callable(d0302.build_mission_packages):\n"
        "    raise SystemExit('repo root bootstrapped data_def.d0302 export missing')\n"
        "if not callable(d0303.build_flight_plans):\n"
        "    raise SystemExit('repo root bootstrapped data_def.d0303 export missing')\n"
        "if not callable(d0304.build_lah_flight_plans_fixed):\n"
        "    raise SystemExit('repo root bootstrapped data_def.d0304 export missing')\n"
        "if not callable(id_allocator.reserve_mission_plan_ids):\n"
        "    raise SystemExit('repo root bootstrapped data_def.id_allocator export missing')\n"
        "original = canonical_config.SEARCH_SPEED_WEIGHT\n"
        "try:\n"
        "    config.SEARCH_SPEED_WEIGHT = 8.75\n"
        "    if canonical_config.SEARCH_SPEED_WEIGHT != 8.75:\n"
        "        raise SystemExit('repo root bootstrapped config assignment forwarding failed')\n"
        "finally:\n"
        "    config.SEARCH_SPEED_WEIGHT = original\n"
    )
    run_python_case("repo-root bootstrapped bare imports", PROJECT_ROOT, code)


def check_mission_dir_bootstrap_imports() -> None:
    code = (
        "import importlib, sys\n"
        "from pathlib import Path\n"
        "project_root = Path.cwd().parents[1]\n"
        "if str(project_root) not in sys.path:\n"
        "    sys.path.insert(0, str(project_root))\n"
        "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
        "runtime.ensure_mission_planner_import_paths(project_root)\n"
        "expected = [\n"
        "    str(project_root),\n"
        "    str(project_root / 'modules'),\n"
        "    str(project_root / 'modules' / 'mission_planning'),\n"
        "    str(project_root / 'modules' / 'mission_planning' / 'MissionPlanner'),\n"
        "]\n"
        "if sys.path[:4] != expected:\n"
        "    raise SystemExit(f'mission dir bootstrap path order changed: {sys.path[:4]!r}')\n"
        "ans = importlib.import_module('AnS')\n"
        "ans_pipeline = importlib.import_module('AnS.mission_pipeline')\n"
        "data_def = importlib.import_module('data_def')\n"
        "config = importlib.import_module('config')\n"
        "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
        "canonical_pipeline = importlib.import_module('modules.mission_planning.MissionPlanner.AnS.mission_pipeline')\n"
        "if ans is not canonical_ans:\n"
        "    raise SystemExit('mission dir bootstrap AnS identity split')\n"
        "if ans_pipeline is not canonical_pipeline:\n"
        "    raise SystemExit('mission dir bootstrap AnS.mission_pipeline identity split')\n"
        "if not callable(data_def.build_lah_flight_plans_fixed):\n"
        "    raise SystemExit('mission dir bootstrap data_def export missing')\n"
        "if not hasattr(config, 'DEFAULT_SWEEP_SEPARATION_M'):\n"
        "    raise SystemExit('mission dir bootstrap config export missing')\n"
    )
    run_python_case("modules/mission_planning bootstrap imports", MISSION_DIR, code)


def check_mission_dir_gui_import() -> None:
    code = (
        "import importlib, os\n"
        "from pathlib import Path\n"
        "start_cwd = Path.cwd()\n"
        "project_root = start_cwd.parents[1]\n"
        "module = importlib.import_module('mission_planning_gui')\n"
        "if Path(module.PROJECT_ROOT) != project_root:\n"
        "    raise SystemExit(f'mission_planning_gui PROJECT_ROOT changed: {module.PROJECT_ROOT!r}')\n"
        "if Path.cwd() != project_root:\n"
        "    raise SystemExit(f'mission_planning_gui bootstrap cwd changed: {Path.cwd()!r}')\n"
        "if os.environ.get('KU_ROLE') != 'mission':\n"
        "    raise SystemExit(f'mission_planning_gui KU_ROLE changed: {os.environ.get(\"KU_ROLE\")!r}')\n"
        "if not callable(getattr(module, '_ensure_mission_planner_import_paths', None)):\n"
        "    raise SystemExit('mission_planning_gui path bootstrap callable missing')\n"
    )
    run_python_case("modules/mission_planning mission_planning_gui import", MISSION_DIR, code)


def check_mission_dir_direct_script_smoke(timeout_s: float) -> None:
    env = base_env()
    env["MISSION_PLANNING_GUI_SMOKE_LAUNCH"] = "1"
    result = subprocess.run(
        [sys.executable, str(GUI_SCRIPT.name)],
        cwd=MISSION_DIR,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if result.returncode != 0:
        fail(
            "mission dir direct script smoke failed with code {}\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.returncode,
                result.stdout,
                result.stderr,
            )
        )
    if "mission_planning_gui smoke launch ok" not in result.stdout:
        fail(
            "mission dir direct script smoke did not report success\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.stdout,
                result.stderr,
            )
        )


def check_mission_planner_legacy_cwd_imports() -> None:
    code = (
        "import importlib, sys\n"
        "from pathlib import Path\n"
        "project_root = Path.cwd().parents[2]\n"
        "if str(project_root) not in sys.path:\n"
        "    sys.path.insert(0, str(project_root))\n"
        "ans = importlib.import_module('AnS')\n"
        "data_def = importlib.import_module('data_def')\n"
        "d0303 = importlib.import_module('data_def.d0303')\n"
        "config = importlib.import_module('config')\n"
        "planning_enhanced = importlib.import_module('planning_enhanced')\n"
        "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
        "canonical_config = importlib.import_module('modules.mission_planning.MissionPlanner.config')\n"
        "if ans is not canonical_ans:\n"
        "    raise SystemExit('MissionPlanner cwd AnS identity split')\n"
        "if not callable(data_def.build_lah_flight_plans_fixed):\n"
        "    raise SystemExit('MissionPlanner cwd data_def package export missing')\n"
        "if not callable(d0303.build_flight_plans):\n"
        "    raise SystemExit('MissionPlanner cwd data_def.d0303 export missing')\n"
        "if config.DEFAULT_SWEEP_SEPARATION_M != canonical_config.DEFAULT_SWEEP_SEPARATION_M:\n"
        "    raise SystemExit('MissionPlanner cwd config constant mismatch')\n"
        "if not callable(planning_enhanced.run_enhanced_divide_and_pattern):\n"
        "    raise SystemExit('MissionPlanner cwd planning_enhanced export missing')\n"
    )
    run_python_case("MissionPlanner legacy cwd imports", MISSION_PLANNER_DIR, code)


def check_dashboard_child_cwd_contract() -> None:
    spec = importlib.util.spec_from_file_location("mission_launch_env_parity_smoke", LAUNCH_ENV_PARITY_SMOKE)
    if spec is None or spec.loader is None:
        fail(f"could not load {LAUNCH_ENV_PARITY_SMOKE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    run_capture = module._capture_run_py_launch()
    app_capture = module._capture_main_window_launch()
    module._check_capture_shape(run_capture)
    module._check_capture_shape(app_capture)
    module._check_cross_launcher_parity(run_capture, app_capture)
    for capture in (run_capture, app_capture):
        cwd = Path(str(capture["popen"]["kwargs"].get("cwd", "")))
        if cwd.resolve() != PROJECT_ROOT.resolve():
            fail(f"{capture['source']} child cwd changed: {cwd}")


def main() -> int:
    try:
        check_project_root_bare_shims_absent()
        check_repo_root_bootstrapped_bare_imports()
        check_mission_dir_bootstrap_imports()
        check_mission_dir_gui_import()
        check_mission_dir_direct_script_smoke(timeout_s=30.0)
        check_mission_planner_legacy_cwd_imports()
        check_dashboard_child_cwd_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("cwd/sys.path/bare import matrix smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
