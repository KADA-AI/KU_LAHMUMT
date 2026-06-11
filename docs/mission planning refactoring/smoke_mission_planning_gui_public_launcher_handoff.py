from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
ENTRYPOINT_PATH = PROJECT_ROOT / "modules" / "mission_planning" / "app" / "gui_entrypoint.py"
PUBLIC_GUI_PATH = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"


def fail(message: str) -> None:
    raise AssertionError(message)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def check_source_contract() -> None:
    entrypoint = read_text(ENTRYPOINT_PATH)
    public_gui = read_text(PUBLIC_GUI_PATH)
    required_entrypoint = (
        'SMOKE_ENV_KEY = "MISSION_PLANNING_GUI_SMOKE_LAUNCH"',
        "def smoke_launch_main(",
        "def run_gui(",
        "def run_public_gui_entrypoint(",
        "gui_module.QApplication",
        "gui_module.MainWindow",
        "gui_module.apply_initial_visibility",
    )
    missing_entrypoint = [marker for marker in required_entrypoint if marker not in entrypoint]
    if missing_entrypoint:
        fail(f"gui_entrypoint.py missing markers: {missing_entrypoint!r}")
    forbidden_entrypoint = ("PyQt5", "import modules.mission_planning.mission_planning_gui")
    present_forbidden = [marker for marker in forbidden_entrypoint if marker in entrypoint]
    if present_forbidden:
        fail(f"gui_entrypoint.py should not import concrete GUI modules: {present_forbidden!r}")

    required_public = (
        "def _smoke_launch_main() -> int:",
        "from modules.mission_planning.app.gui_entrypoint import smoke_launch_main",
        "return smoke_launch_main(sys.modules[__name__])",
        "from modules.mission_planning.app.gui_entrypoint import run_public_gui_entrypoint",
        "sys.exit(run_public_gui_entrypoint(sys.modules[__name__]))",
    )
    missing_public = [marker for marker in required_public if marker not in public_gui]
    if missing_public:
        fail(f"mission_planning_gui.py missing public launcher handoff markers: {missing_public!r}")


def check_lazy_import_contract() -> None:
    code = (
        "import importlib, sys\n"
        "entry = importlib.import_module('modules.mission_planning.app.gui_entrypoint')\n"
        "if 'modules.mission_planning.mission_planning_gui' in sys.modules:\n"
        "    raise SystemExit('gui_entrypoint imported public GUI module')\n"
        "if any(name == 'PyQt5' or name.startswith('PyQt5.') for name in sys.modules):\n"
        "    raise SystemExit('gui_entrypoint imported PyQt5')\n"
        "for attr in ('smoke_launch_main', 'run_gui', 'run_public_gui_entrypoint'):\n"
        "    if not callable(getattr(entry, attr, None)):\n"
        "        raise SystemExit(f'missing callable {attr}')\n"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if completed.returncode != 0:
        fail(
            "gui_entrypoint lazy import contract failed\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                completed.stdout,
                completed.stderr,
            )
        )


def check_fake_module_handoff() -> None:
    entry = importlib.import_module("modules.mission_planning.app.gui_entrypoint")
    calls: list[str] = []

    class FakeQMainWindow:
        pass

    class FakeMainWindow(FakeQMainWindow):
        def __init__(self) -> None:
            calls.append("MainWindow")

    class FakeApp:
        def __init__(self, argv: list[str]) -> None:
            self.argv = list(argv)
            calls.append("QApplication")

        def quit(self) -> None:
            calls.append("quit")

        def exec_(self) -> int:
            calls.append("exec")
            return 0

    class FakeQApplication:
        _instance = None

        def __new__(cls, argv: list[str]) -> FakeApp:
            app = FakeApp(argv)
            cls._instance = app
            return app

        @classmethod
        def instance(cls):
            return cls._instance

    class FakeQTimer:
        @staticmethod
        def singleShot(_delay_ms: int, callback) -> None:
            calls.append("singleShot")
            callback()

    def fake_stylesheet(_app, _root) -> None:
        calls.append("stylesheet")

    def fake_signature() -> str:
        calls.append("signature")
        return "signature"

    def fake_visibility(_app, _win, _positioner) -> None:
        calls.append("visibility")

    fake_module = SimpleNamespace(
        QApplication=FakeQApplication,
        QMainWindow=FakeQMainWindow,
        MainWindow=FakeMainWindow,
        QTimer=FakeQTimer,
        PROJECT_ROOT=PROJECT_ROOT,
        load_shared_stylesheet=fake_stylesheet,
        _planner_runtime_source_signature=fake_signature,
        apply_initial_visibility=fake_visibility,
        position_window_from_env=object(),
    )

    FakeQApplication._instance = None
    result = entry.run_public_gui_entrypoint(
        fake_module,
        argv=["mission_planning_gui.py"],
        environ={entry.SMOKE_ENV_KEY: "1"},
    )
    if result != 0:
        fail(f"smoke handoff returned {result}")
    for marker in ("QApplication", "stylesheet", "signature", "singleShot", "quit", "exec"):
        if marker not in calls:
            fail(f"smoke handoff did not call {marker}: {calls!r}")

    calls.clear()
    FakeQApplication._instance = None
    result = entry.run_public_gui_entrypoint(
        fake_module,
        argv=["mission_planning_gui.py"],
        environ={},
    )
    if result != 0:
        fail(f"normal handoff returned {result}")
    for marker in ("QApplication", "stylesheet", "MainWindow", "visibility", "exec"):
        if marker not in calls:
            fail(f"normal handoff did not call {marker}: {calls!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke mission_planning_gui public launcher handoff.")
    parser.parse_args()

    try:
        check_source_contract()
        check_lazy_import_contract()
        check_fake_module_handoff()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("mission_planning_gui public launcher handoff smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
