from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("KU_ROLE", "mission")


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def with_temp_db(callback: Callable[[Path], None]) -> None:
    from modules.common import db_paths

    original_get_db_subpath = db_paths.get_db_subpath
    tmp_root = Path(tempfile.mkdtemp(prefix="mp_0902_replay_store_"))
    try:
        def fake_get_db_subpath(*parts: object) -> Path:
            return tmp_root.joinpath(*(str(part) for part in parts))

        db_paths.get_db_subpath = fake_get_db_subpath
        callback(tmp_root)
    finally:
        cleanup_process_console_state()
        db_paths.get_db_subpath = original_get_db_subpath
        shutil.rmtree(tmp_root, ignore_errors=True)


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


def sample_payload(
    timestamp: int,
    *,
    reason: str = "sample replan",
    level: int = 3,
    detail_type: str = "communicationLossRTB",
) -> dict[str, Any]:
    return {
        "timestamp": int(timestamp),
        "replanRequestTime": {"replanRequestTimestamp": int(timestamp)},
        "replanLevel": int(level),
        "replanRequest": reason,
        "replanDetail": {"trigger": "0401", "triggerType": detail_type, "missionPlanID": 700000001},
        "optionList": [{"missionPlanID": 700000101, "optionName": "option-a"}],
    }


def check_transport_store(tmp_root: Path) -> None:
    from modules.common import replan_request_transport_store as store

    previous_mode = os.environ.get("REPLAN_0902_SIDECAR_MODE")
    try:
        os.environ["REPLAN_0902_SIDECAR_MODE"] = "compact"
        payload = sample_payload(833886405198, reason="reason-a", level=3)
        expected_path = tmp_root / "DSS_Internal" / "replan_request_transport" / "replan_request_833886405198.json"
        if store.payload_path_for_payload(payload) != expected_path:
            fail("0902 sidecar payload path contract changed")

        saved_path = store.save_payload(payload)
        if saved_path != expected_path or not expected_path.exists():
            fail(f"0902 sidecar save path changed: {saved_path!r}")
        entries = json.loads(expected_path.read_text(encoding="utf-8"))
        if not isinstance(entries, list) or len(entries) != 1:
            fail(f"0902 sidecar compact entries shape changed: {entries!r}")

        duplicate_path = store.save_payload(dict(payload))
        if duplicate_path != expected_path:
            fail("0902 sidecar duplicate save returned a different path")
        entries = json.loads(expected_path.read_text(encoding="utf-8"))
        if len(entries) != 1:
            fail(f"0902 sidecar duplicate identity dedupe changed: {entries!r}")

        payload_b = sample_payload(833886405198, reason="reason-b", level=4)
        store.save_payload(payload_b)
        entries = json.loads(expected_path.read_text(encoding="utf-8"))
        if len(entries) != 2:
            fail(f"0902 sidecar multi-entry append changed: {entries!r}")
        if store.load_payload(833886405198, reason="reason-a", replan_level=3).get("replanRequest") != "reason-a":
            fail("0902 sidecar filtered load changed for first entry")
        if store.load_payload(833886405198, reason="missing")["replanRequest"] != "reason-b":
            fail("0902 sidecar fallback-to-last load changed")

        latest_payload = sample_payload(833886405299, reason="latest", level=5)
        store.save_payload(latest_payload)
        latest = store.load_latest_payload()
        if not latest or latest.get("replanRequest") != "latest":
            fail(f"0902 sidecar latest load changed: {latest!r}")

        os.environ["REPLAN_0902_SIDECAR_MODE"] = "off"
        if store.sidecar_enabled():
            fail("0902 sidecar off mode no longer disables sidecar")
        if store.save_payload(sample_payload(833886405300)) is not None:
            fail("0902 sidecar off mode save no longer returns None")
    finally:
        if previous_mode is None:
            os.environ.pop("REPLAN_0902_SIDECAR_MODE", None)
        else:
            os.environ["REPLAN_0902_SIDECAR_MODE"] = previous_mode


def check_gui_capture(tmp_root: Path) -> None:
    import importlib

    gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
    previous_mode = os.environ.get("REPLAN_0902_SIDECAR_MODE")
    try:
        os.environ["REPLAN_0902_SIDECAR_MODE"] = "pretty"
        window = gui.MainWindow.__new__(gui.MainWindow)
        window._events = []
        window._logs = []
        window._record_replan_timing_event = (
            lambda name, **kwargs: window._events.append((name, kwargs))
        )
        window._append_log_line = lambda text: window._logs.append(str(text))

        ctx = {"reason": "reason-a", "replan_level": 3, "plan_ids": [700000101], "mission_ids": [1], "option_names": ["option-a"]}
        payload = sample_payload(833886405400, reason="reason-a", level=3)
        window._capture_replan_payload_for_replay(payload, ctx)

        capture_path = Path(ctx.get("_0902_capture_path", ""))
        expected_path = tmp_root / "DSS_Internal" / "replan_request_transport" / "replan_request_833886405400.json"
        if capture_path != expected_path or not expected_path.exists():
            fail(f"0902 GUI sidecar capture path changed: {capture_path!r}")
        if not window._events or window._events[-1][0] != "0902_archived":
            fail(f"0902 GUI capture timing event changed: {window._events!r}")
    finally:
        if previous_mode is None:
            os.environ.pop("REPLAN_0902_SIDECAR_MODE", None)
        else:
            os.environ["REPLAN_0902_SIDECAR_MODE"] = previous_mode


def check_generic_replan_store(tmp_root: Path) -> None:
    from modules.mission_planning.runtime import next_collab_replan_store, replan_store

    detail_path = replan_store.save_detail(
        "unit_replan",
        "unit_detail",
        700000501,
        {"kind": "generic"},
    )
    expected = tmp_root / "DSS_Internal" / "unit_replan" / "unit_detail_700000501.json"
    if detail_path != expected or not expected.exists():
        fail(f"generic replan detail path changed: {detail_path!r}")
    loaded = replan_store.load_detail("unit_replan", "unit_detail", 700000501)
    if not loaded or loaded.get("kind") != "generic" or loaded.get("missionPlanID") != 700000501:
        fail(f"generic replan detail load changed: {loaded!r}")
    if replan_store.load_detail("unit_replan", "unit_detail", 999) is not None:
        fail("generic replan missing detail no longer returns None")

    event_path = replan_store.save_event("unit_replan", "stage/name with spaces", {"value": 1})
    if event_path.parent != tmp_root / "DSS_Internal" / "unit_replan":
        fail(f"generic replan event parent changed: {event_path!r}")
    if not event_path.name.startswith("stage_name_with_spaces_"):
        fail(f"generic replan event stage sanitization changed: {event_path.name!r}")

    next_path = next_collab_replan_store.save_detail(700000601, {"kind": "next"})
    expected_next = tmp_root / "DSS_Internal" / "next_collab_replan" / "next_collab_detail_700000601.json"
    if next_path != expected_next:
        fail(f"next-collab detail wrapper path changed: {next_path!r}")
    next_loaded = next_collab_replan_store.load_detail(700000601)
    if not next_loaded or next_loaded.get("kind") != "next":
        fail(f"next-collab detail wrapper load changed: {next_loaded!r}")


def check_common_trigger_detail_stores(tmp_root: Path) -> None:
    from modules.common import (
        imaging_schedule_replan_store,
        path_deviation_replan_store,
        prior_replan_store,
    )

    cases = (
        (
            "prior",
            prior_replan_store,
            tmp_root / "DSS_Internal" / "prior_replan" / "prior_detail_700000701.json",
        ),
        (
            "imaging",
            imaging_schedule_replan_store,
            tmp_root / "DSS_Internal" / "imaging_schedule_replan" / "imaging_schedule_detail_700000702.json",
        ),
        (
            "path",
            path_deviation_replan_store,
            tmp_root / "DSS_Internal" / "path_deviation_replan" / "path_deviation_detail_700000703.json",
        ),
    )
    for idx, (label, module, expected_path) in enumerate(cases, start=1):
        mission_plan_id = 700000700 + idx
        saved = module.save_detail(mission_plan_id, {"kind": label})
        if saved != expected_path or not expected_path.exists():
            fail(f"{label} detail store path changed: {saved!r}")
        loaded = module.load_detail(mission_plan_id)
        if not loaded or loaded.get("kind") != label or loaded.get("missionPlanID") != mission_plan_id:
            fail(f"{label} detail store load changed: {loaded!r}")


def main() -> int:
    try:
        with_temp_db(check_transport_store)
        with_temp_db(check_gui_capture)
        with_temp_db(check_generic_replan_store)
        with_temp_db(check_common_trigger_detail_stores)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 replay/store-backed detail smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
