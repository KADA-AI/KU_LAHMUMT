from __future__ import annotations

import argparse
import importlib
import json
import os
import socket
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]

ROLE_CONTRACTS = {
    "mission": {
        "script_name": "mission_planning_gui.py",
        "script_path": PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py",
        "port": "45981",
        "offset": "40,40",
        "title": "KU Mission Planning Console",
        "listener_default": "env_ctrl_port(45981)",
        "control_role": 'role="mission"',
    },
    "monitor": {
        "script_name": "monitoring_gui.py",
        "script_path": PROJECT_ROOT / "modules" / "monitoring" / "monitoring_gui.py",
        "port": "45982",
        "offset": "130,90",
        "title": "KU Monitoring Console",
        "listener_default": "env_ctrl_port(45982)",
        "control_role": 'role="monitor"',
    },
    "decision": {
        "script_name": "decision_support_gui.py",
        "script_path": PROJECT_ROOT / "modules" / "decision_support" / "decision_support_gui.py",
        "port": "45983",
        "offset": "220,140",
        "title": "KU Decision Support Console",
        "listener_default": "env_ctrl_port(45983)",
        "control_role": 'role="decision"',
    },
    "info": {
        "script_name": "info_manage.py",
        "script_path": PROJECT_ROOT / "modules" / "info_manage" / "info_manage.py",
        "port": "45984",
        "offset": "310,190",
        "title": "KU Info Manage Console",
        "listener_default": "env_ctrl_port(45984)",
        "control_role": 'role="info"',
    },
}


class SmokeFailure(RuntimeError):
    pass


class _FakeTextLine:
    def __init__(self, text: str = "") -> None:
        self._text = text

    def text(self) -> str:
        return self._text


class _FakeProcess:
    _next_pid = 930000

    def __init__(self) -> None:
        type(self)._next_pid += 1
        self.pid = type(self)._next_pid

    def poll(self) -> None:
        return None

    def terminate(self) -> None:
        return None

    def kill(self) -> None:
        return None

    def wait(self, timeout: float | None = None) -> int:
        return 0


class _FakeSocket:
    sent_packets: list[tuple[bytes, tuple[str, int]]] = []
    closed_count = 0

    def __init__(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def sendto(self, data: bytes, address: tuple[str, int]) -> int:
        self.sent_packets.append((bytes(data), address))
        return len(data)

    def close(self) -> None:
        type(self).closed_count += 1


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
    db_paths = importlib.import_module("modules.common.db_paths")
    with _patched_many(
        [
            (process_console, "ensure_console", lambda *_args, **_kwargs: False),
            (process_console, "install_process_file_logging", lambda *_args, **_kwargs: None),
            (db_paths, "bootstrap_db_root", lambda: PROJECT_ROOT / "Logs"),
        ]
    ):
        return importlib.import_module("run")


def _role_for_script(script_path: Path) -> str:
    for role, contract in ROLE_CONTRACTS.items():
        if script_path.resolve() == Path(contract["script_path"]).resolve():
            return role
    _fail(f"unexpected launch script: {script_path}")
    return ""


def _capture_run_py_cold_start() -> tuple[Any, list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], Any]:
    run_module = _import_run_module_safely()
    popen_calls: list[dict[str, Any]] = []
    flags_calls: list[dict[str, Any]] = []
    timer_calls: list[dict[str, Any]] = []

    def fake_popen(args: list[str], **kwargs: Any) -> _FakeProcess:
        proc = _FakeProcess()
        popen_calls.append({"args": list(args), "kwargs": dict(kwargs), "proc": proc})
        return proc

    def fake_creationflags_for_subprocess(**kwargs: Any) -> int:
        flags_calls.append(dict(kwargs))
        return 0

    def fake_single_shot(delay_ms: int, callback: Any) -> None:
        timer_calls.append({"delay_ms": int(delay_ms), "callback": callback})

    fake_window = SimpleNamespace(
        _role_processes={},
        _db_path_line=_FakeTextLine(str(PROJECT_ROOT / "Logs")),
        validate_launch_prerequisites=lambda **_kwargs: {"ok": True},
    )
    fake_orchestrator = SimpleNamespace(
        win=fake_window,
        widgets={},
        _module_mode={role: "초기화 모드" for role in ROLE_CONTRACTS},
        _mode_text="초기화 모드",
        _safe_log=lambda _message: None,
        _refresh_service_status_panel=lambda: None,
        _set_mode_text_all=lambda text: setattr(fake_orchestrator, "_mode_text", text),
    )
    fake_orchestrator._ensure_launch_ready = lambda **_kwargs: True
    fake_orchestrator._launch_gui = (
        lambda script_name, **kwargs: run_module.DashboardOrchestrator._launch_gui(
            fake_orchestrator,
            script_name,
            **kwargs,
        )
    )

    with _patched_many(
        [
            (subprocess, "Popen", fake_popen),
            (run_module, "kill_python_processes_for_scripts", lambda *_args, **_kwargs: []),
            (run_module, "preferred_console_python", lambda executable=None: str(executable or sys.executable)),
            (run_module, "should_show_module_consoles", lambda: False),
            (run_module, "creationflags_for_subprocess", fake_creationflags_for_subprocess),
            (run_module.QTimer, "singleShot", fake_single_shot),
        ]
    ):
        run_module.DashboardOrchestrator._launch_all_guis(fake_orchestrator)

    return run_module, popen_calls, flags_calls, timer_calls, fake_window


def _check_cold_start_launches(popen_calls: list[dict[str, Any]], flags_calls: list[dict[str, Any]]) -> None:
    if len(popen_calls) != len(ROLE_CONTRACTS):
        _fail(f"cold start captured {len(popen_calls)} Popen calls, expected {len(ROLE_CONTRACTS)}")

    seen_roles: set[str] = set()
    for call in popen_calls:
        args = call["args"]
        kwargs = call["kwargs"]
        if len(args) < 2:
            _fail(f"cold start launch args too short: {args!r}")
        script_path = Path(args[1])
        role = _role_for_script(script_path)
        seen_roles.add(role)
        contract = ROLE_CONTRACTS[role]
        if script_path.resolve() != Path(contract["script_path"]).resolve():
            _fail(f"{role} script path changed: {script_path}")
        if Path(str(kwargs.get("cwd", ""))).resolve() != PROJECT_ROOT.resolve():
            _fail(f"{role} launch cwd changed: {kwargs.get('cwd')!r}")
        if kwargs.get("shell") is True:
            _fail(f"{role} launch must use shell=False/default")
        if kwargs.get("start_new_session") is not True:
            _fail(f"{role} launch start_new_session changed: {kwargs.get('start_new_session')!r}")

        env = kwargs.get("env")
        if not isinstance(env, dict):
            _fail(f"{role} launch did not pass env")
        expected_env = {
            "KU_LAUNCHED_BY_DASHBOARD": "1",
            "KU_CTRL_PORT": contract["port"],
            "KU_START_HIDDEN": "1",
            "KU_HIDE_ON_CLOSE": "1",
            "KU_VIEWER_ONLY": "0",
            "KU_WINDOW_OFFSET": contract["offset"],
            "KU_CONSOLE_TITLE": contract["title"],
            "PYTHONUNBUFFERED": "1",
        }
        for key, expected in expected_env.items():
            if env.get(key) != expected:
                _fail(f"{role} env {key} changed: {env.get(key)!r} != {expected!r}")
        if not str(env.get("KU_MISSION_DB_ROOT") or "").strip():
            _fail(f"{role} KU_MISSION_DB_ROOT missing")

    if seen_roles != set(ROLE_CONTRACTS):
        _fail(f"cold start roles changed: {sorted(seen_roles)!r}")

    if len(flags_calls) != len(ROLE_CONTRACTS):
        _fail(f"creationflags calls changed: {len(flags_calls)}")
    for flags in flags_calls:
        if flags.get("show_console") is not False or flags.get("new_process_group") is not True:
            _fail(f"cold start creationflags contract changed: {flags!r}")


def _check_process_registry(fake_window: Any, popen_calls: list[dict[str, Any]]) -> None:
    role_processes = getattr(fake_window, "_role_processes", None)
    if not isinstance(role_processes, dict):
        _fail("fake launch window did not keep role process registry")
    for call in popen_calls:
        role = _role_for_script(Path(call["args"][1]))
        if role_processes.get(role) is not call["proc"]:
            _fail(f"{role} process was not registered under role key")


def _check_timer_contract(timer_calls: list[dict[str, Any]]) -> None:
    delays = [entry["delay_ms"] for entry in timer_calls]
    if delays != [1000, 1000, 1200]:
        _fail(f"cold start post-launch timers changed: {delays!r}")
    if not all(callable(entry["callback"]) for entry in timer_calls):
        _fail("cold start scheduled a non-callable timer callback")


def _check_control_sender(run_module: Any) -> None:
    fake_orchestrator = SimpleNamespace(_safe_log=lambda _message: None)
    _FakeSocket.sent_packets.clear()
    _FakeSocket.closed_count = 0
    with _patched_many([(socket, "socket", _FakeSocket)]):
        for role in ROLE_CONTRACTS:
            ok = run_module.DashboardOrchestrator._send_role_ctrl(
                fake_orchestrator,
                role,
                {"cmd": "show_window"},
            )
            if ok is not True:
                _fail(f"_send_role_ctrl returned false for {role}")

    if len(_FakeSocket.sent_packets) != len(ROLE_CONTRACTS):
        _fail(f"control sender packet count changed: {len(_FakeSocket.sent_packets)}")

    for data, address in _FakeSocket.sent_packets:
        if address[0] != "127.0.0.1":
            _fail(f"control sender host changed: {address!r}")
        payload = json.loads(data.decode("utf-8"))
        role = payload.get("role")
        if role not in ROLE_CONTRACTS:
            _fail(f"control sender role changed: {payload!r}")
        expected_port = int(ROLE_CONTRACTS[role]["port"])
        if int(address[1]) != expected_port:
            _fail(f"control sender port changed for {role}: {address[1]} != {expected_port}")
        if payload.get("cmd") != "show_window":
            _fail(f"control sender payload command changed: {payload!r}")


def _check_control_sender_real_udp(run_module: Any) -> None:
    fake_orchestrator = SimpleNamespace(_safe_log=lambda _message: None)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        sock.settimeout(2.0)
        port = int(sock.getsockname()[1])
        ok = run_module.DashboardOrchestrator._send_role_ctrl(
            fake_orchestrator,
            "mission",
            {"cmd": "show_window"},
            port=port,
        )
        if ok is not True:
            _fail("_send_role_ctrl returned false for real UDP smoke")
        data, address = sock.recvfrom(8192)
    finally:
        sock.close()

    if address[0] != "127.0.0.1":
        _fail(f"real UDP control sender host changed: {address!r}")
    payload = json.loads(data.decode("utf-8"))
    if payload != {"cmd": "show_window", "role": "mission"}:
        _fail(f"real UDP control payload changed: {payload!r}")


def _check_gui_listener_contracts() -> None:
    for role, contract in ROLE_CONTRACTS.items():
        path = Path(contract["script_path"])
        if not path.exists():
            _fail(f"{role} GUI script missing: {path}")
        text = path.read_text(encoding="utf-8", errors="ignore")
        if "start_ctrl_listener" not in text:
            _fail(f"{role} GUI no longer starts a control listener")
        if str(contract["listener_default"]) not in text:
            _fail(f"{role} GUI listener default port changed")
        if "handle_window_control" not in text:
            _fail(f"{role} GUI no longer handles window control")
        if str(contract["control_role"]) not in text:
            _fail(f"{role} GUI window control role changed")


def _check_gui_process_control_contract() -> None:
    gui_process_control = importlib.import_module("modules.common.gui_process_control")

    class FakeWindow:
        def __init__(self) -> None:
            self.calls: list[str] = []

        def show(self) -> None:
            self.calls.append("show")

        def hide(self) -> None:
            self.calls.append("hide")

        def windowState(self) -> int:
            return 0

        def setWindowState(self, _state: int) -> None:
            self.calls.append("setWindowState")

        def raise_(self) -> None:
            self.calls.append("raise")

        def activateWindow(self) -> None:
            self.calls.append("activate")

    alias_cases = {
        "mission": "assignment",
        "monitor": "monitoring",
        "decision": "decision_support",
        "info": "info_manage",
    }
    for role, alias in alias_cases.items():
        window = FakeWindow()
        handled = gui_process_control.handle_window_control(
            window,
            {"cmd": "show_window", "target": alias},
            role=role,
        )
        if handled is not True:
            _fail(f"window control alias not handled for {role}/{alias}")
        for expected_call in ("show", "raise", "activate"):
            if expected_call not in window.calls:
                _fail(f"window control show contract missing {expected_call} for {role}")

        wrong = FakeWindow()
        handled = gui_process_control.handle_window_control(
            wrong,
            {"cmd": "show_window", "target": "wrong-role"},
            role=role,
        )
        if handled is not False or wrong.calls:
            _fail(f"window control target filter changed for {role}")

        hidden = FakeWindow()
        handled = gui_process_control.handle_window_control(
            hidden,
            {"cmd": "hide_window", "role": role},
            role=role,
        )
        if handled is not True or hidden.calls != ["hide"]:
            _fail(f"window control hide contract changed for {role}: {hidden.calls!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke run.py cold-start GUI launch and control-port contracts.")
    parser.parse_args()

    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        os.environ.setdefault("KU_SHOW_RUN_CONSOLE", "1")
        os.environ.setdefault("KU_SHOW_MODULE_CONSOLES", "0")
        run_module, popen_calls, flags_calls, timer_calls, fake_window = _capture_run_py_cold_start()
        _check_cold_start_launches(popen_calls, flags_calls)
        _check_process_registry(fake_window, popen_calls)
        _check_timer_contract(timer_calls)
        _check_control_sender(run_module)
        _check_control_sender_real_udp(run_module)
        _check_gui_listener_contracts()
        _check_gui_process_control_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("run.py cold-start GUI/control-port smoke ok")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
