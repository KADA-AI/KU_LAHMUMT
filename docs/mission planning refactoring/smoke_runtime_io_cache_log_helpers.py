from __future__ import annotations

import json
import os
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def fail(message: str) -> None:
    raise AssertionError(message)


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


@contextmanager
def patched_env(name: str, value: str) -> Iterator[None]:
    old_value = os.environ.get(name)
    os.environ[name] = value
    try:
        yield
    finally:
        if old_value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = old_value


@contextmanager
def patched_ensure_db_payload(base_dir: Path) -> Iterator[None]:
    from modules.common import db_paths

    original = db_paths.ensure_db_payload

    def _fake_ensure(name: str) -> Path:
        target = base_dir / str(name)
        target.mkdir(parents=True, exist_ok=True)
        return target

    db_paths.ensure_db_payload = _fake_ensure  # type: ignore[assignment]
    try:
        yield
    finally:
        db_paths.ensure_db_payload = original  # type: ignore[assignment]


@contextmanager
def captured_process_events() -> Iterator[list[tuple[str, str]]]:
    from modules.common import process_console

    captured: list[tuple[str, str]] = []
    original = process_console.emit_process_log
    process_console.emit_process_log = lambda module, text: captured.append((str(module), str(text)))  # type: ignore[assignment]
    try:
        yield captured
    finally:
        process_console.emit_process_log = original  # type: ignore[assignment]


def check_json_io(tmp_root: Path) -> None:
    from modules.mission_planning.runtime import json_io

    compact = json_io.dumps_json({"b": 2, "a": 1}, pretty=False, sort_keys=True)
    expect_equal("compact JSON sort order", compact.decode("utf-8"), '{"a":1,"b":2}')

    flight_path_dir = tmp_root / "FlightPath"
    path = flight_path_dir / "100000001.json"
    payload = {
        "Source": "upper-only",
        "aircraftID": 9,
        "waypointList": [{"waypointID": 1, "speed": 40}],
    }
    expect_true("FlightPath write should create file", json_io.write_json(path, payload, pretty=True))
    written = load_json(path)
    expect_equal("FlightPath Source normalized to source", written.get("source"), "upper-only")
    expect_true("FlightPath Source key removed", "Source" not in written)

    unchanged = json_io.write_json(path, payload, pretty=True)
    expect_equal("unchanged FlightPath write skipped", unchanged, False)
    expect_true("FlightPath tmp file removed", not path.with_suffix(path.suffix + ".tmp").exists())

    bytes_path = tmp_root / "bytes" / "payload.json"
    raw_payload = b'{"k":1}'
    expect_true("write_json_bytes creates file", json_io.write_json_bytes(bytes_path, raw_payload))
    expect_equal("write_json_bytes content", bytes_path.read_bytes(), raw_payload)
    expect_equal("write_json_bytes unchanged skipped", json_io.write_json_bytes(bytes_path, raw_payload), False)
    expect_true("write_json_bytes tmp file removed", not bytes_path.with_suffix(bytes_path.suffix + ".tmp").exists())

    preserve_path = tmp_root / "FlightPath_preserve_source.json"
    preserve_payload = {
        "Source": "upper",
        "source": "lower",
        "waypointList": [{"waypointID": 2}],
    }
    expect_true("FlightPath stem target write", json_io.write_json(preserve_path, preserve_payload))
    preserved = load_json(preserve_path)
    expect_equal("existing lower source preserved", preserved.get("source"), "lower")
    expect_true("duplicate Source removed when source exists", "Source" not in preserved)

    batch_messages: list[str] = []
    batch_path = tmp_root / "batch" / "a.json"
    first = json_io.write_json_batch([(batch_path, {"v": 1})], log=batch_messages.append)
    second = json_io.write_json_batch([(batch_path, {"v": 1})], log=batch_messages.append)
    expect_equal("batch first write flag", first[0]["written"], True)
    expect_equal("batch second write flag", second[0]["written"], False)
    expect_equal("batch second skip flag", second[0]["skipped"], True)
    expect_true("batch log written", any("written" in message for message in batch_messages))
    expect_true("batch log unchanged", any("unchanged" in message for message in batch_messages))


def check_latest_input_cache(tmp_root: Path) -> None:
    from modules.mission_planning.runtime.cache import latest_input

    latest_input.reset_latest_inputs()
    expect_equal(
        "latest cache reset description",
        latest_input.describe_latest_ids(),
        "0201:ID=-,ts=-,src=-, 0203:ID=-,ts=-,src=-",
    )
    latest_input.update_from_payload(
        "0201",
        {
            "inputMissionPackageID": "42",
            "timestamp": "101",
            "source": "lower-source",
            "payloadMarker": "0201",
        },
    )
    latest_input.update_from_payload(
        "0203",
        {
            "MISSIONREFERENCEPACKAGEID": "77",
            "TIMESTAMP": "202",
            "Source": "upper-source",
        },
    )
    latest_input.update_from_payload("0202", {"packageID": 99})

    expect_equal("0201 latest package ID", latest_input.get_latest_package_id("0201"), 42)
    expect_equal("0203 latest package ID", latest_input.get_latest_package_id("0203"), 77)
    expect_equal("unsupported latest package ID", latest_input.get_latest_package_id("0202"), None)

    snapshot = latest_input.get_latest_snapshot("0201")
    expect_equal("0201 snapshot source", snapshot.source, "lower-source")
    snapshot.payload["payloadMarker"] = "mutated-copy"
    expect_equal(
        "latest snapshot top-level payload copy",
        latest_input.get_latest_snapshot("0201").payload.get("payloadMarker"),
        "0201",
    )

    package_dir = tmp_root / "InputMissionPlan"
    package_dir.mkdir(parents=True)
    existing = package_dir / "42.json"
    existing.write_text("{}", encoding="utf-8")
    expect_equal("latest input path resolver", latest_input.resolve_path_from_cache("0201", package_dir), existing)
    expect_equal("latest input resolver unsupported", latest_input.resolve_path_from_cache("9999", package_dir), None)

    try:
        latest_input.get_latest_snapshot("9999")
    except KeyError:
        pass
    else:
        fail("unsupported latest snapshot must raise KeyError")

    latest_input.reset_latest_inputs()
    expect_equal("latest cache reset package ID", latest_input.get_latest_package_id("0201"), None)


def check_mission_plan_file_logger(tmp_root: Path) -> None:
    from modules.mission_planning.runtime.logging.plan_file_logger import MissionPlanFileLogger

    db_root = tmp_root / "db"
    with patched_ensure_db_payload(db_root), patched_env("REPLAN_RUNTIME_ARTIFACT_MODE", "pretty"):
        logger = MissionPlanFileLogger()
        run = logger.start_run(
            {
                "plan_ids": [700001, "bad", 700002],
                "mission_ids": [1, 2],
                "replanLevel": "2",
                "contextSet": {3, 4},
            },
            "unit-replan",
            session_id="session-1",
        )
        expect_true("file logger start_run returns run", run is not None)
        expect_true("file logger active run set", logger.current_run() is run)
        assert run is not None
        expect_equal("file logger plan IDs", run.plan_ids, [700001, 700002])
        expect_equal("file logger replan level", run.replan_level, 2)

        run.add_message("hello")
        run.add_step("build", "ok", detail={"path": tmp_root / "artifact.json"}, message="built")
        run.add_issue("warn", "warning message", {"nested": object()})
        run.set_stop_reason("done")
        written = [Path(path) for path in run.finalize("ok", summary={"complete": True})]
        expect_equal("file logger written count", len(written), 2)
        expect_equal("file logger first path", written[0].name, "missionPlan_700001.json")
        expect_equal("file logger second path", written[1].name, "missionPlan_700002.json")

        logged = load_json(written[0])
        expect_equal("file logger status", logged["status"], "ok")
        expect_equal("file logger reason", logged["reason"], "unit-replan")
        expect_equal("file logger session ID", logged["sessionId"], "session-1")
        expect_equal("file logger mission plan ID", logged["missionPlanID"], 700001)
        expect_equal("file logger stop reason", logged["stopReason"], "done")
        expect_equal("file logger summary flag", logged["summary"]["complete"], True)
        expect_equal("file logger step name", logged["steps"][0]["step"], "build")
        expect_equal("file logger issue code", logged["issues"][0]["code"], "warn")

        duplicate = logger.start_run({"plan_ids": [700001]}, "duplicate-run", session_id="session-dup")
        expect_true("duplicate run returns run", duplicate is not None)
        assert duplicate is not None
        duplicate_written = [Path(path) for path in duplicate.finalize("ok")]
        expect_equal("duplicate run written count", len(duplicate_written), 1)
        expect_true("duplicate log tokenized path", duplicate_written[0].name.startswith("missionPlan_700001_"))
        expect_true("duplicate log keeps primary file", written[0].exists())

        pending = logger.start_run({}, "pending-run", session_id="session-pending")
        expect_true("pending run returns run", pending is not None)
        assert pending is not None
        pending_written = [Path(path) for path in pending.finalize("ok")]
        expect_equal("pending run written count", len(pending_written), 1)
        expect_true("pending log path", pending_written[0].name.startswith("missionPlan_pending_"))
        expect_equal("pending missionPlanID", load_json(pending_written[0])["missionPlanID"], None)

        logger.clear_active()
        expect_equal("file logger clear active", logger.current_run(), None)

        logger.log_blocked({"plan_ids": [700003], "replan_level": 5}, "blocked-reason", "blocked message")
        expect_equal("file logger blocked clears active", logger.current_run(), None)
        blocked_path = db_root / "DSS_Internal" / "missionPlan_700003.json"
        blocked = load_json(blocked_path)
        expect_equal("blocked log status", blocked["status"], "blocked")
        expect_equal("blocked log reason", blocked["reason"], "blocked-reason")
        expect_equal("blocked log summary", blocked["summary"]["blocked"], "blocked message")


class _FakeLogTab:
    def __init__(self) -> None:
        self.events: list[tuple[str, Any, Any, Any]] = []

    def start_session(self, session_id: str, meta: dict[str, Any]) -> None:
        self.events.append(("start", session_id, meta, None))

    def append_event(
        self,
        session_id: str,
        level: str,
        message: str,
        *,
        detail: Any = None,
        timestamp: Any = None,
    ) -> None:
        self.events.append(("append", session_id, level, message, detail, timestamp))

    def finish_session(self, session_id: str, status: str, summary: Any = None) -> None:
        self.events.append(("finish", session_id, status, summary))


def check_pipeline_logging_helpers() -> None:
    from modules.mission_planning.runtime.logging import pipeline_events

    expect_equal("checkpoint phases include delivery", "delivery" in pipeline_events.REPLAN_CHECKPOINT_PHASES, True)
    expect_equal("outcome failure", pipeline_events.infer_checkpoint_outcome("algorithm_fail"), "failure")
    expect_equal("outcome skipped", pipeline_events.infer_checkpoint_outcome("delivery_suppress"), "skipped")
    expect_equal("outcome ok", pipeline_events.infer_checkpoint_outcome("queued_done"), "ok")
    expect_equal("outcome checkpoint", pipeline_events.infer_checkpoint_outcome("source_loaded"), "checkpoint")
    expect_equal("normalize 0902", pipeline_events.normalize_replan_checkpoint("0902 received"), "0902_received")
    expect_equal("normalize source", pipeline_events.normalize_replan_checkpoint("source artifacts loaded"), "source_artifacts_loaded")
    expect_equal("normalize reserve", pipeline_events.normalize_replan_checkpoint("id_reserve_done"), "id_reserve")
    expect_equal("normalize write", pipeline_events.normalize_replan_checkpoint("persist output"), "write")
    expect_equal("normalize delivery", pipeline_events.normalize_replan_checkpoint("0301 queued"), "delivery")
    expect_equal("normalize fallback", pipeline_events.normalize_replan_checkpoint("fallback noop"), "fallback_noop_failure")

    timer = pipeline_events.PipelinePhaseTimer(pipeline="unit", replan_transaction_id="txn", emit_events=False)
    timer.mark("load")
    snapshot = timer.snapshot(include_total=True)
    expect_true("timer load key", "load" in snapshot)
    expect_true("timer total key", "total" in snapshot)
    expect_true("timer nonnegative total", snapshot["total"] >= 0.0)

    with captured_process_events() as captured:
        pipeline_events.emit_pipeline_event(
            event="unit_event",
            module="mission_planning",
            process_id=123,
            thread_name="worker",
            replan_transaction_id="txn",
            pipeline="unit",
            phase="build",
            mission_plan_id=700001,
            elapsed_ms=1.25,
            outcome="ok",
            extra={"custom": "value"},
        )
        pipeline_events.emit_replan_checkpoint(
            name="source artifacts loaded",
            replan_transaction_id="txn",
            pipeline="unit",
            mission_plan_id=700001,
        )

    expect_equal("captured process event count", len(captured), 2)
    prefix = "[REPLAN][EVENT] "
    module, text = captured[0]
    expect_equal("captured process module", module, "mission_planning")
    expect_true("captured process prefix", text.startswith(prefix))
    event_payload = json.loads(text[len(prefix):])
    expect_equal("event payload event", event_payload["event"], "unit_event")
    expect_equal("event payload process ID", event_payload["processId"], 123)
    expect_equal("event payload custom", event_payload["custom"], "value")

    checkpoint_payload = json.loads(captured[1][1][len(prefix):])
    expect_equal("checkpoint event name", checkpoint_payload["event"], "replan_checkpoint")
    expect_equal("checkpoint phase", checkpoint_payload["phase"], "source_artifacts_loaded")
    expect_equal("checkpoint outcome", checkpoint_payload["outcome"], "checkpoint")
    expect_equal("checkpoint detail", checkpoint_payload["checkpoint"], "source artifacts loaded")

    tab = _FakeLogTab()
    emitted: list[dict[str, Any]] = []
    manager = pipeline_events.PipelineLogManager(
        emit_callback=emitted.append,
        log_tab_provider=lambda: tab,
        sanitize_reason=lambda value, fallback: str(value or fallback).strip() or fallback,
    )
    session_id = manager.open_session({"plan_ids": [1], "mission_ids": [2], "replan_level": 3}, " reason ")
    expect_equal("log manager session ID", session_id, "run-0001")
    manager.log_event(session_id, "info", "message", detail={"x": 1})
    manager.close_session(session_id, "done", summary={"ok": True})
    expect_equal("log manager emitted count", len(emitted), 3)

    for payload in emitted:
        manager.handle_event(payload)
    expect_equal("log manager tab start", tab.events[0][0], "start")
    expect_equal("log manager tab append", tab.events[1][0], "append")
    expect_equal("log manager tab finish", tab.events[2][0], "finish")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="mp_runtime_io_cache_log_") as tmp:
        tmp_root = Path(tmp)
        check_json_io(tmp_root)
        check_latest_input_cache(tmp_root)
        check_mission_plan_file_logger(tmp_root)
    check_pipeline_logging_helpers()
    print("runtime I/O/cache/log helper smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
