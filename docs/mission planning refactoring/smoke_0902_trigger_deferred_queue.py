from __future__ import annotations

import ast
import copy
import os
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("KU_ROLE", "mission")


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def payload(trigger: str = "", trigger_type: str = "") -> dict[str, Any]:
    return {"replanDetail": {"trigger": trigger, "triggerType": trigger_type}}


def make_window(gui: Any):
    window = gui.MainWindow.__new__(gui.MainWindow)
    window._initplan_running = False
    window._deferred_replan_requests = []
    window._active_plan_context = {}
    window._smoke_events = []
    window._smoke_logs = []
    window._smoke_schedules = []

    def record(event_name: str, **kwargs: Any) -> None:
        window._smoke_events.append((event_name, copy.deepcopy(kwargs)))

    def append_log(text: str) -> None:
        window._smoke_logs.append(str(text))

    def schedule(delay_ms: int = 1000) -> None:
        window._smoke_schedules.append(int(delay_ms))

    window._record_replan_timing_event = record
    window._append_log_line = append_log
    window._schedule_replan_pipeline = schedule
    return window


def check_trigger_delay_values(gui: Any) -> None:
    window = make_window(gui)
    original_load_runtime_settings = gui.load_runtime_settings

    try:
        gui.load_runtime_settings = lambda: {"values": {}}
        cases = (
            ("collab default", payload("0402", "collabReexecuteInputRefresh"), 30),
            ("attack 0402", payload("0402", ""), 0),
            ("attack closed destroyed", payload("0201", "attackClosedDestroyed"), 0),
            ("communication loss", payload("0401", "communicationLossRTB"), 55_000),
            ("unknown", payload("0401", "unknown"), 100),
            ("missing detail", {}, 100),
        )
        for label, data, expected in cases:
            actual = window._replan_delay_ms_for_payload(data)
            if actual != expected:
                fail(f"0902 trigger delay changed for {label}: {actual!r} != {expected!r}")

        gui.load_runtime_settings = lambda: {
            "values": {"replan_collab_reexecute_schedule_delay_ms": "42.9"}
        }
        if window._replan_delay_ms_for_payload(payload("0402", "collabReexecuteInputRefresh")) != 42:
            fail("0902 collab delay runtime override int(float(...)) behavior changed")

        gui.load_runtime_settings = lambda: {
            "values": {"replan_collab_reexecute_schedule_delay_ms": "-5"}
        }
        if window._replan_delay_ms_for_payload(payload("0402", "collabReexecuteInputRefresh")) != 0:
            fail("0902 collab delay negative runtime override clamp changed")

        gui.load_runtime_settings = lambda: {
            "values": {"replan_collab_reexecute_schedule_delay_ms": "bad"}
        }
        try:
            window._replan_delay_ms_for_payload(payload("0402", "collabReexecuteInputRefresh"))
        except ValueError:
            pass
        else:
            fail("0902 collab delay invalid runtime override no longer raises ValueError")
    finally:
        gui.load_runtime_settings = original_load_runtime_settings


def check_deferred_queue_order_and_copy(gui: Any) -> None:
    window = make_window(gui)
    original_monotonic = gui.time.monotonic
    try:
        gui.time.monotonic = lambda: 100.0
        slow_ctx = {"plan_ids": [700000200], "nested": {"value": "original"}}
        window._queue_deferred_replan_request(slow_ctx, delay_ms=2000)
        slow_ctx["nested"]["value"] = "mutated"
        window._queue_deferred_replan_request({"plan_ids": [700000100]}, delay_ms=500)
        window._queue_deferred_replan_request({"plan_ids": [700000000]}, delay_ms=-10)
    finally:
        gui.time.monotonic = original_monotonic

    queue = window._deferred_replan_requests
    plan_order = [item["ctx"]["plan_ids"][0] for item in queue]
    if plan_order != [700000000, 700000100, 700000200]:
        fail(f"0902 deferred queue due_at ordering changed: {plan_order!r}")
    if queue[2]["ctx"]["nested"]["value"] != "original":
        fail("0902 deferred queue no longer deep-copies ctx")

    queued_events = [event for event in window._smoke_events if event[0] == "deferred_queued"]
    if [event[1]["extra"]["delay_ms"] for event in queued_events] != [2000, 500, 0]:
        fail(f"0902 deferred queue delay normalization changed: {queued_events!r}")
    if [event[1]["extra"]["queued"] for event in queued_events] != [1, 2, 3]:
        fail(f"0902 deferred queue count reporting changed: {queued_events!r}")
    if not any("deferred while replan pipeline running" in text for text in window._smoke_logs):
        fail("0902 deferred queue log text changed")


def check_resume_deferred_queue(gui: Any) -> None:
    window = make_window(gui)
    window._deferred_replan_requests = [
        {"ctx": {"plan_ids": [700000501]}, "due_at": 105.0},
        {"ctx": {"plan_ids": [700000502]}, "due_at": 110.0},
    ]
    original_monotonic = gui.time.monotonic
    try:
        gui.time.monotonic = lambda: 103.2
        window._resume_deferred_replan_request()
    finally:
        gui.time.monotonic = original_monotonic

    if window._active_plan_context != {"plan_ids": [700000501]}:
        fail(f"0902 deferred resume active context changed: {window._active_plan_context!r}")
    if [item["ctx"]["plan_ids"][0] for item in window._deferred_replan_requests] != [700000502]:
        fail(f"0902 deferred resume queue pop changed: {window._deferred_replan_requests!r}")
    if window._smoke_schedules != [1800]:
        fail(f"0902 deferred resume remaining delay schedule changed: {window._smoke_schedules!r}")

    resumed_events = [event for event in window._smoke_events if event[0] == "deferred_resumed"]
    if len(resumed_events) != 1:
        fail(f"0902 deferred resume event count changed: {resumed_events!r}")
    if resumed_events[0][1]["extra"] != {"delay_ms": 1800, "queued": 1}:
        fail(f"0902 deferred resume event payload changed: {resumed_events!r}")

    blocked = make_window(gui)
    blocked._initplan_running = True
    blocked._deferred_replan_requests = [{"ctx": {"plan_ids": [1]}, "due_at": 101.0}]
    blocked._resume_deferred_replan_request()
    if blocked._smoke_schedules or len(blocked._deferred_replan_requests) != 1:
        fail("0902 deferred resume no longer waits while init planning is running")


def check_handle_replan_consumes_computed_delay() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    source = gui_path.read_text(encoding="utf-8-sig", errors="ignore")
    tree = ast.parse(source)
    target = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_handle_replan_received_impl":
            target = node
            break
    if target is None:
        fail("mission_planning_gui missing _handle_replan_received_impl")

    text = ast.get_source_segment(source, target) or ""
    required_fragments = (
        "delay_ms = self._replan_delay_ms_for_payload(payload)",
        "self._queue_deferred_replan_request(ctx, delay_ms=delay_ms)",
        "self._schedule_replan_pipeline(delay_ms=delay_ms)",
    )
    for fragment in required_fragments:
        if fragment not in text:
            fail(f"0902 handler no longer consumes computed delay through {fragment}")
    if text.index(required_fragments[0]) > text.index(required_fragments[1]):
        fail("0902 handler queues deferred request before computing delay")
    if text.index(required_fragments[0]) > text.index(required_fragments[2]):
        fail("0902 handler schedules immediate request before computing delay")


def main() -> int:
    try:
        import importlib

        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_trigger_delay_values(gui)
        check_deferred_queue_order_and_copy(gui)
        check_resume_deferred_queue(gui)
        check_handle_replan_consumes_computed_delay()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 trigger deferred queue smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
