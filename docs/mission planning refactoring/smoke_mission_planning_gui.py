from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GUI_SCRIPT = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"


def _base_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    env.setdefault("KU_ROLE", "mission")
    env["PYTHONPATH"] = str(PROJECT_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    return env


def _run_subprocess(label: str, args: list[str], *, code: str | None = None, timeout_s: float = 30.0) -> None:
    completed = subprocess.run(
        args if code is None else [sys.executable, "-c", code],
        cwd=PROJECT_ROOT,
        env=_base_env(),
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "{} failed with code {}\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                label,
                completed.returncode,
                completed.stdout,
                completed.stderr,
            )
        )


def run_import_smoke(timeout_s: float) -> None:
    code = (
        "import importlib, os, sys\n"
        "from pathlib import Path\n"
        f"project_root = Path({str(PROJECT_ROOT)!r})\n"
        "if str(project_root) not in sys.path:\n"
        "    sys.path.insert(0, str(project_root))\n"
        "os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')\n"
        "module = importlib.import_module('modules.mission_planning.mission_planning_gui')\n"
        "from PyQt5.QtWidgets import QMainWindow\n"
        "if os.environ.get('KU_ROLE') != 'mission':\n"
        "    raise SystemExit(f'KU_ROLE changed: {os.environ.get(\"KU_ROLE\")!r}')\n"
        "if Path(module.PROJECT_ROOT) != project_root:\n"
        "    raise SystemExit(f'PROJECT_ROOT mismatch: {module.PROJECT_ROOT!r}')\n"
        "if not issubclass(module.MainWindow, QMainWindow):\n"
        "    raise SystemExit('MainWindow is not QMainWindow')\n"
        "for attr in ('_smoke_launch_main', '_planner_runtime_source_signature', '_ensure_mission_planner_import_paths'):\n"
        "    if not callable(getattr(module, attr, None)):\n"
        "        raise SystemExit(f'missing callable {attr}')\n"
        "if not module._planner_runtime_source_signature():\n"
        "    raise SystemExit('planner runtime source signature is empty')\n"
        "print('mission_planning_gui import smoke ok')\n"
    )
    _run_subprocess("mission_planning_gui import smoke", [], code=code, timeout_s=timeout_s)


def run_launch_smoke(timeout_s: float) -> None:
    env = _base_env()
    env["MISSION_PLANNING_GUI_SMOKE_LAUNCH"] = "1"
    completed = subprocess.run(
        [sys.executable, str(GUI_SCRIPT)],
        cwd=PROJECT_ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "mission_planning_gui launch smoke failed with code {}\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                completed.returncode,
                completed.stdout,
                completed.stderr,
            )
        )
    if "mission_planning_gui smoke launch ok" not in completed.stdout:
        raise RuntimeError(
            "mission_planning_gui launch smoke did not report success\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                completed.stdout,
                completed.stderr,
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke import and launch mission_planning_gui.py.")
    parser.add_argument("--mode", choices=("all", "import", "launch"), default="all")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()

    try:
        if args.mode in ("all", "import"):
            run_import_smoke(args.timeout_s)
        if args.mode in ("all", "launch"):
            run_launch_smoke(args.timeout_s)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(f"mission_planning_gui {args.mode} smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
