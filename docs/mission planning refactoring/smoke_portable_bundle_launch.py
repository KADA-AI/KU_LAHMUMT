from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
BUNDLE_ROOT = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/portable_mission_bundle"


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    path = PROJECT_ROOT / rel_path
    text = path.read_text(encoding="utf-8", errors="ignore")
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing portable launch markers: {missing!r}")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def stop_process_tree(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    else:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def wait_json(url: str, proc: subprocess.Popen, *, timeout_s: float = 45.0) -> dict:
    deadline = time.time() + timeout_s
    last_error: str | None = None
    while time.time() < deadline:
        if proc.poll() is not None:
            output = ""
            try:
                output = proc.stdout.read() if proc.stdout is not None else ""
            except Exception:
                output = ""
            fail(f"portable process exited before {url}: exit={proc.returncode}, output={output!r}")
        try:
            with urllib.request.urlopen(url, timeout=2) as response:
                return json.loads(response.read().decode("utf-8"))
        except Exception as exc:
            last_error = str(exc)
            time.sleep(0.5)
    fail(f"portable endpoint did not respond: {url}; last_error={last_error}")
    return {}


def launch_and_probe(label: str, command: Sequence[str]) -> tuple[dict, dict]:
    port = free_port()
    env = dict(os.environ)
    env["MISSION_APP_HOST"] = "127.0.0.1"
    env["MISSION_APP_PORT"] = str(port)
    proc = subprocess.Popen(
        list(command),
        cwd=BUNDLE_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        health = wait_json(f"http://127.0.0.1:{port}/api/health", proc)
        model = wait_json(f"http://127.0.0.1:{port}/api/model", proc)
    finally:
        stop_process_tree(proc)

    expect_true(f"{label} health ok", health.get("ok") is True)
    expect_true(f"{label} model exists", health.get("model_exists") is True)
    expect_true(f"{label} model info exists", model.get("model_exists") is True)
    expect_true(f"{label} model file", model.get("model_file") == "latest_model.zip")
    expect_true(f"{label} config file", model.get("config_file") == "model_config.json")
    expect_true(f"{label} action labels", isinstance(model.get("action_labels"), list))
    return health, model


def check_source_contracts() -> None:
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
        "@echo off",
        "setlocal",
        'cd /d "%~dp0"',
        "python app.py",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/web.py",
        "def create_app(bundle_root: Path | None = None) -> Flask:",
        '@app.get("/api/health")',
        '@app.get("/api/model")',
        "service = MissionService(root)",
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/service.py",
        'self.model_path = self.models_dir / "latest_model.zip"',
        'self.config_path = self.models_dir / "model_config.json"',
        "def health(self) -> Dict[str, Any]:",
        "def model_info(self) -> Dict[str, Any]:",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke portable mission bundle python/bat launch.")
    parser.parse_args()

    try:
        if not BUNDLE_ROOT.exists():
            fail(f"missing portable bundle root: {BUNDLE_ROOT}")
        check_source_contracts()
        launch_and_probe("python app.py", [sys.executable, "app.py"])
        if os.name == "nt":
            launch_and_probe("run_portable.bat", ["cmd.exe", "/c", "run_portable.bat"])
        else:
            launch_and_probe("run_portable fallback", [sys.executable, "app.py"])
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("portable bundle launch smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
