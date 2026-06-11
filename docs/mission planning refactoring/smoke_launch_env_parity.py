from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GUI_SCRIPT = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"

COMMON_ENV_CONTRACT = {
    "KU_CTRL_PORT": "45981",
    "KU_START_HIDDEN": "1",
    "KU_HIDE_ON_CLOSE": "1",
    "KU_VIEWER_ONLY": "0",
    "KU_WINDOW_OFFSET": "40,40",
    "KU_CONSOLE_TITLE": "KU Mission Planning Console",
    "PYTHONUNBUFFERED": "1",
}


class SmokeFailure(RuntimeError):
    pass


class _FakeTextLine:
    def __init__(self, text: str = "") -> None:
        self._text = text

    def text(self) -> str:
        return self._text


class _FakeProcess:
    pid = 919191

    def poll(self) -> None:
        return None

    def terminate(self) -> None:
        return None

    def kill(self) -> None:
        return None

    def wait(self, timeout: float | None = None) -> int:
        return 0


@contextmanager
def _patched(obj: Any, name: str, value: Any) -> Iterator[None]:
    original = getattr(obj, name)
    setattr(obj, name, value)
    try:
        yield
    finally:
        setattr(obj, name, original)


@contextmanager
def _patched_many(patches: list[tuple[Any, str, Any]]) -> Iterator[None]:
    originals: list[tuple[Any, str, Any]] = []
    try:
        for obj, name, value in patches:
            originals.append((obj, name, getattr(obj, name)))
            setattr(obj, name, value)
        yield
    finally:
        for obj, name, original in reversed(originals):
            setattr(obj, name, original)


def _fail(message: str) -> None:
    raise SmokeFailure(message)


def _ensure_project_path() -> None:
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))


def _import_run_module_safely():
    _ensure_project_path()
    process_console = importlib.import_module("modules.common.process_console")
    with _patched_many(
        [
            (process_console, "ensure_console", lambda *_args, **_kwargs: False),
            (process_console, "install_process_file_logging", lambda *_args, **_kwargs: None),
        ]
    ):
        return importlib.import_module("run")


def _capture_run_py_launch() -> dict[str, Any]:
    run_module = _import_run_module_safely()
    popen_calls: list[dict[str, Any]] = []
    flags_calls: list[dict[str, Any]] = []

    def fake_popen(args: list[str], **kwargs: Any) -> _FakeProcess:
        popen_calls.append({"args": list(args), "kwargs": dict(kwargs)})
        return _FakeProcess()

    def fake_creationflags_for_subprocess(**kwargs: Any) -> int:
        flags_calls.append(dict(kwargs))
        return 0

    fake_window = SimpleNamespace(
        _role_processes={},
        _db_path_line=_FakeTextLine(""),
    )
    fake_orchestrator = SimpleNamespace(
        win=fake_window,
        _ensure_launch_ready=lambda **_kwargs: True,
        _safe_log=lambda _message: None,
        _refresh_service_status_panel=lambda: None,
    )

    with _patched_many(
        [
            (subprocess, "Popen", fake_popen),
            (run_module, "kill_python_processes_for_scripts", lambda *_args, **_kwargs: []),
            (run_module, "preferred_console_python", lambda executable=None: str(executable or sys.executable)),
            (run_module, "should_show_module_consoles", lambda: False),
            (run_module, "creationflags_for_subprocess", fake_creationflags_for_subprocess),
        ]
    ):
        run_module.DashboardOrchestrator._launch_gui(
            fake_orchestrator,
            "mission_planning_gui.py",
            start_hidden=True,
        )

    if len(popen_calls) != 1:
        _fail(f"run.py launch captured {len(popen_calls)} Popen calls, expected 1")

    return {
        "source": "run.py",
        "popen": popen_calls[0],
        "creationflags": flags_calls,
    }


def _capture_main_window_launch() -> dict[str, Any]:
    _ensure_project_path()
    main_window = importlib.import_module("app.ui.main_window")
    popen_calls: list[dict[str, Any]] = []
    flags_calls: list[dict[str, Any]] = []

    def fake_popen(args: list[str], **kwargs: Any) -> _FakeProcess:
        popen_calls.append({"args": list(args), "kwargs": dict(kwargs)})
        return _FakeProcess()

    def fake_creationflags_for_subprocess(**kwargs: Any) -> int:
        flags_calls.append(dict(kwargs))
        return 0

    fake_log = SimpleNamespace(append_log=lambda _message: None)
    fake_window = SimpleNamespace(
        _role_processes={},
        module_mission=fake_log,
        validate_launch_prerequisites=lambda **_kwargs: {"ok": True},
        _log_to_modules=lambda _message: None,
        _debug_log=lambda _message: None,
        _schedule_module_powerup=lambda _role: None,
    )

    with _patched_many(
        [
            (subprocess, "Popen", fake_popen),
            (main_window, "kill_python_processes_for_scripts", lambda *_args, **_kwargs: []),
            (main_window, "preferred_console_python", lambda executable=None: str(executable or sys.executable)),
            (main_window, "should_show_module_consoles", lambda: False),
            (main_window, "creationflags_for_subprocess", fake_creationflags_for_subprocess),
        ]
    ):
        main_window.MainWindow._launch_role(
            fake_window,
            "mission",
            schedule_powerup=False,
            start_hidden=True,
        )

    if len(popen_calls) != 1:
        _fail(f"app/ui/main_window.py launch captured {len(popen_calls)} Popen calls, expected 1")

    return {
        "source": "app/ui/main_window.py",
        "popen": popen_calls[0],
        "creationflags": flags_calls,
    }


def _env(capture: dict[str, Any]) -> dict[str, str]:
    env = capture["popen"]["kwargs"].get("env")
    if not isinstance(env, dict):
        _fail(f"{capture['source']} did not pass an env dict to Popen")
    return env


def _check_capture_shape(capture: dict[str, Any]) -> None:
    source = capture["source"]
    popen = capture["popen"]
    args = popen["args"]
    kwargs = popen["kwargs"]
    if len(args) != 2:
        _fail(f"{source} mission launch args changed: {args!r}")
    script = Path(args[1])
    if script.resolve() != GUI_SCRIPT.resolve():
        _fail(f"{source} mission script changed: {script}")
    cwd = Path(str(kwargs.get("cwd", "")))
    if cwd.resolve() != PROJECT_ROOT.resolve():
        _fail(f"{source} cwd changed: {cwd}")
    if kwargs.get("shell") is True:
        _fail(f"{source} must launch mission_planning_gui.py with shell=False/default")

    env = _env(capture)
    for key, expected in COMMON_ENV_CONTRACT.items():
        actual = env.get(key)
        if actual != expected:
            _fail(f"{source} env {key} changed: {actual!r} != {expected!r}")

    launched_by_dashboard = env.get("KU_LAUNCHED_BY_DASHBOARD")
    if launched_by_dashboard not in (None, "1"):
        _fail(f"{source} KU_LAUNCHED_BY_DASHBOARD changed: {launched_by_dashboard!r}")
    mission_db_root = env.get("KU_MISSION_DB_ROOT")
    if mission_db_root is not None and not str(mission_db_root).strip():
        _fail(f"{source} KU_MISSION_DB_ROOT is present but empty")


def _check_cross_launcher_parity(run_capture: dict[str, Any], app_capture: dict[str, Any]) -> None:
    run_env = _env(run_capture)
    app_env = _env(app_capture)
    for key in COMMON_ENV_CONTRACT:
        if run_env.get(key) != app_env.get(key):
            _fail(f"launch env parity mismatch for {key}: {run_env.get(key)!r} != {app_env.get(key)!r}")

    run_flags = run_capture["creationflags"]
    app_flags = app_capture["creationflags"]
    if not run_flags:
        _fail("run.py did not call creationflags_for_subprocess")
    if not app_flags:
        _fail("app/ui/main_window.py did not call creationflags_for_subprocess")
    if run_flags[0].get("new_process_group") is not True:
        _fail(f"run.py mission launch process-group contract changed: {run_flags[0]!r}")
    if app_flags[0].get("new_process_group", False):
        _fail(f"app/ui/main_window.py mission launch process-group contract changed: {app_flags[0]!r}")


def _run_gui_smoke_with_launch_env(label: str, env: dict[str, str], timeout_s: float) -> None:
    smoke_env = dict(env)
    smoke_env["MISSION_PLANNING_GUI_SMOKE_LAUNCH"] = "1"
    smoke_env["QT_QPA_PLATFORM"] = "offscreen"
    completed = subprocess.run(
        [sys.executable, str(GUI_SCRIPT)],
        cwd=PROJECT_ROOT,
        env=smoke_env,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if completed.returncode != 0:
        _fail(
            "{} env launch smoke failed with code {}\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                label,
                completed.returncode,
                completed.stdout,
                completed.stderr,
            )
        )
    if "mission_planning_gui smoke launch ok" not in completed.stdout:
        _fail(
            "{} env launch smoke did not report success\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                label,
                completed.stdout,
                completed.stderr,
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke mission GUI launch-env parity across dashboard launchers.")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()

    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        os.environ.setdefault("KU_SHOW_RUN_CONSOLE", "1")
        os.environ.setdefault("KU_SHOW_MODULE_CONSOLES", "0")

        run_capture = _capture_run_py_launch()
        app_capture = _capture_main_window_launch()
        _check_capture_shape(run_capture)
        _check_capture_shape(app_capture)
        _check_cross_launcher_parity(run_capture, app_capture)
        _run_gui_smoke_with_launch_env("run.py", _env(run_capture), args.timeout_s)
        _run_gui_smoke_with_launch_env("app/ui/main_window.py", _env(app_capture), args.timeout_s)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("mission launch env parity smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
