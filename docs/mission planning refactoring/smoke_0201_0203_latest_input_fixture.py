from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures" / "payloads"


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


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


def load_fixture(name: str) -> dict[str, Any]:
    path = FIXTURE_DIR / name
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        fail(f"fixture is not an object: {path}")
    return payload


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        fail(f"JSON file is not an object: {path}")
    return payload


def cleanup_process_console_state() -> None:
    try:
        from modules.common import process_console

        tee_type = getattr(process_console, "_TeeStream", None)
        for stream_name in ("stdout", "stderr"):
            stream = getattr(sys, stream_name)
            if tee_type is not None and isinstance(stream, tee_type):
                setattr(sys, stream_name, getattr(stream, "_original", stream))
        sinks = list(getattr(process_console, "_ACTIVE_SINKS", {}).values())
        for sink in sinks:
            try:
                sink.close()
            except Exception:
                pass
        try:
            process_console._ACTIVE_SINKS.clear()
        except Exception:
            pass
        time.sleep(0.05)
    except Exception:
        pass


def with_temp_db(callback: Callable[[Path], None]) -> None:
    from modules.common import db_paths

    original_get_active_db_root = db_paths.get_active_db_root
    tmp_root = Path(tempfile.mkdtemp(prefix="mp_latest_input_"))
    try:
        def fake_get_active_db_root() -> Path:
            tmp_root.mkdir(parents=True, exist_ok=True)
            return tmp_root

        db_paths.get_active_db_root = fake_get_active_db_root  # type: ignore[assignment]
        callback(tmp_root)
    finally:
        cleanup_process_console_state()
        db_paths.get_active_db_root = original_get_active_db_root  # type: ignore[assignment]
        shutil.rmtree(tmp_root, ignore_errors=True)


class _LogSignal:
    def __init__(self, window: Any) -> None:
        self._window = window

    def emit(self, text: object) -> None:
        self._window._smoke_logs.append(str(text))


def _copy_submit_payload(kwargs: dict[str, Any]) -> dict[str, Any]:
    copied: dict[str, Any] = {}
    for key, value in kwargs.items():
        if isinstance(value, dict):
            copied[key] = {
                str(inner_key): set(inner_value)
                if isinstance(inner_value, set)
                else copy.deepcopy(inner_value)
                for inner_key, inner_value in value.items()
            }
        else:
            copied[key] = copy.deepcopy(value)
    return copied


def make_window(gui: Any) -> Any:
    window = gui.MainWindow.__new__(gui.MainWindow)
    window._last_logged_input_ids = {"0201": None, "0203": None}
    window._session_scope = gui.MainWindow._create_empty_scope(window)
    window._plan_status = "before-planning"
    window._smoke_logs: list[str] = []
    window._smoke_refreshes = 0
    window._smoke_schedules: list[str] = []
    window._smoke_id_updates: list[dict[str, Any]] = []
    window.log_sig = _LogSignal(window)
    window._refresh_input_banner = lambda: setattr(
        window,
        "_smoke_refreshes",
        window._smoke_refreshes + 1,
    )
    window._schedule_planner_warmup = lambda reason: window._smoke_schedules.append(str(reason))
    window._submit_id_tab_update = lambda **kwargs: window._smoke_id_updates.append(
        _copy_submit_payload(kwargs)
    )
    return window


def check_helper_boundaries() -> None:
    from modules.mission_planning.app.message_handlers.input_packages import (
        extract_payload_source,
        payload_has_core_data,
        prepare_cached_payload_for_file,
    )

    if payload_has_core_data("0202", {"inputMissionList": [{"id": 1}]}):
        fail("0202 is no longer excluded from latest input core-data handling")
    if prepare_cached_payload_for_file("0202", 2202, {"inputMissionList": [{"id": 1}]}) is not None:
        fail("0202 is no longer excluded from cached payload preparation")

    empty_0201 = {
        "inputMissionPackageID": 1200,
        "inputMissionList": [],
        "availableAircraftList": [],
    }
    if payload_has_core_data("0201", empty_0201):
        fail("0201 empty core lists are now treated as materializable")
    if prepare_cached_payload_for_file("0201", 1200, empty_0201) is not None:
        fail("0201 empty core lists are now prepared for file materialization")

    only_aircraft_0201 = {
        "inputMissionPackageID": 1202,
        "inputMissionList": [],
        "availableAircraftList": [{"aircraftID": 1}],
    }
    prepared_0201 = prepare_cached_payload_for_file("0201", 1202, only_aircraft_0201)
    if prepared_0201 is None or prepared_0201[:2] != ("InputMissionPlan", 1202):
        fail(f"0201 one-core-list materialization changed: {prepared_0201!r}")

    empty_0203 = {
        "missionReferencePackageID": 3200,
        "takeOverInfoList": [],
        "flightAreaList": [],
        "handOverInfoList": [],
    }
    if payload_has_core_data("0203", empty_0203):
        fail("0203 empty core lists are now treated as materializable")
    if prepare_cached_payload_for_file("0203", 3200, empty_0203) is not None:
        fail("0203 empty core lists are now prepared for file materialization")

    only_handover_0203 = {
        "missionReferencePackageID": 3204,
        "takeOverInfoList": [],
        "flightAreaList": [],
        "handOverInfoList": [{"aircraftID": 1}],
    }
    prepared_0203 = prepare_cached_payload_for_file("0203", 3204, only_handover_0203)
    if prepared_0203 is None or prepared_0203[:2] != ("MissionReferenceInfo", 3204):
        fail(f"0203 one-core-list materialization changed: {prepared_0203!r}")

    source_marker = {"raw": "source-object"}
    if extract_payload_source({"source": source_marker}) is not source_marker:
        fail("source extraction no longer preserves the truthy lower-case source object")
    expect_equal(
        "Source precedence and raw whitespace",
        extract_payload_source({"Source": "  UPPER RAW  ", "source": "lower"}),
        "  UPPER RAW  ",
    )


def check_gui_latest_input_flow(tmp_root: Path) -> None:
    import importlib

    gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
    gui.reset_latest_inputs()
    window = make_window(gui)

    try:
        unsupported_payload = {
            "packageID": 2202,
            "Source": "RAW_0202",
            "inputMissionList": [{"inputMissionID": 1}],
        }
        gui.MainWindow._handle_latest_input_payload(window, "0202", unsupported_payload)
        expect_equal("0202 latest package", gui.get_latest_package_id("0202"), None)
        expect_equal("0202 update logs", window._smoke_logs, [])
        expect_equal("0202 warmup schedules", window._smoke_schedules, [])

        empty_0201 = {
            "inputMissionPackageID": 1200,
            "timestamp": 1000,
            "Source": "EMPTY_0201",
            "inputMissionList": [],
            "availableAircraftList": [],
        }
        gui.MainWindow._handle_latest_input_payload(window, "0201", empty_0201)
        expect_equal("0201 empty payload cache ID", gui.get_latest_package_id("0201"), 1200)
        gui.MainWindow._prime_latest_input_file(window, "0201")
        expect_true(
            "0201 empty core list should not materialize",
            not (tmp_root / "InputMissionPlan" / "1200.json").exists(),
        )

        p0201 = load_fixture("sample_0201.json")
        p0201["inputMissionPackageID"] = 1201
        p0201["timestamp"] = 1001
        p0201["Source"] = "  RAW_SOURCE_0201  "
        gui.MainWindow._handle_latest_input_payload(window, "0201", p0201)
        expect_equal("0201 latest package", gui.get_latest_package_id("0201"), 1201)
        expect_true(
            "0201 raw Source log text",
            any("source=  RAW_SOURCE_0201  " in line for line in window._smoke_logs),
        )
        expect_true("0201 warmup scheduled", "0201_updated" in window._smoke_schedules)
        gui.MainWindow._prime_latest_input_file(window, "0201")
        path_0201 = tmp_root / "InputMissionPlan" / "1201.json"
        expect_true("0201 non-empty core payload materialized", path_0201.exists())
        written_0201 = load_json(path_0201)
        expect_equal("0201 materialized Source raw value", written_0201.get("Source"), "  RAW_SOURCE_0201  ")
        last_0201_update = window._smoke_id_updates[-1]
        expect_equal("0201 id-tab cmpk update", last_0201_update.get("cmpk_id"), 1201)
        expect_true(
            "0201 id-tab package scope",
            1201 in last_0201_update.get("scope", {}).get("packages", set()),
        )

        empty_0203 = {
            "missionReferencePackageID": 3200,
            "timestamp": 2000,
            "source": "EMPTY_0203",
            "takeOverInfoList": [],
            "flightAreaList": [],
            "handOverInfoList": [],
        }
        gui.MainWindow._handle_latest_input_payload(window, "0203", empty_0203)
        expect_equal("0203 empty payload cache ID", gui.get_latest_package_id("0203"), 3200)
        gui.MainWindow._prime_latest_input_file(window, "0203")
        expect_true(
            "0203 empty core list should not materialize",
            not (tmp_root / "MissionReferenceInfo" / "3200.json").exists(),
        )

        p0203 = load_fixture("sample_0203.json")
        p0203["missionReferencePackageID"] = 3203
        p0203["timestamp"] = 2003
        p0203["source"] = "  raw-source-0203  "
        gui.MainWindow._handle_latest_input_payload(window, "0203", p0203)
        expect_equal("0203 latest package", gui.get_latest_package_id("0203"), 3203)
        expect_true(
            "0203 raw source log text",
            any("source=  raw-source-0203  " in line for line in window._smoke_logs),
        )
        expect_true("0203 warmup scheduled", "0203_updated" in window._smoke_schedules)
        gui.MainWindow._prime_latest_input_file(window, "0203")
        path_0203 = tmp_root / "MissionReferenceInfo" / "3203.json"
        expect_true("0203 non-empty core payload materialized", path_0203.exists())
        expect_true(
            "0203 must not materialize under InputMissionPlan",
            not (tmp_root / "InputMissionPlan" / "3203.json").exists(),
        )
        written_0203 = load_json(path_0203)
        expect_equal("0203 materialized source raw value", written_0203.get("source"), "  raw-source-0203  ")
        last_0203_update = window._smoke_id_updates[-1]
        expect_equal("0203 id-tab mrpk update", last_0203_update.get("mrpk_id"), 3203)

        banner_text, banner_tip = gui.MainWindow._build_input_banner_info(window)
        for expected in (
            "0201: 1201",
            "InputMissionPlan/1201.json",
            "0203: 3203",
            "MissionReferenceInfo/3203.json",
        ):
            if expected not in banner_text:
                fail(f"latest input banner text missing {expected!r}: {banner_text!r}")
        for expected_path in (path_0201, path_0203):
            if str(expected_path) not in banner_tip:
                fail(f"latest input banner tooltip missing {expected_path!s}: {banner_tip!r}")

        expect_true("latest input banner refreshed", window._smoke_refreshes >= 5)
    finally:
        gui.reset_latest_inputs()


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke 0201/0203 latest input handling.")
    parser.parse_args()

    try:
        configure_import_paths()
        check_helper_boundaries()
        with_temp_db(check_gui_latest_input_flow)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0201/0203 latest input fixture smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
