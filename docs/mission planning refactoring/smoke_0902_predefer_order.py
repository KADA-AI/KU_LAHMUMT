from __future__ import annotations

import ast
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


def sample_payload() -> dict[str, Any]:
    return {
        "replanLevel": 3,
        "optionList": [
            {"missionPlanID": 700000101, "optionName": "option-a"},
            {"missionPlanID": 700000102, "optionName": "option-b"},
            {"missionPlanID": 700000103, "optionName": "option-c"},
        ],
        "replanDetail": {
            "trigger": "0401",
            "triggerType": "communicationLossRTB",
        },
        "replanRequest": "baseline replan",
    }


def check_handler_source_order() -> None:
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
    fragments = {
        "timing": "self._start_replan_timing(ctx, payload)",
        "terrain": "self._schedule_replan_terrain_warmup(ctx, payload)",
        "delay": "delay_ms = self._replan_delay_ms_for_payload(payload)",
        "init_branch": 'if getattr(self, "_initplan_running", False):',
        "queue": "self._queue_deferred_replan_request(ctx, delay_ms=delay_ms)",
        "schedule": "self._schedule_replan_pipeline(delay_ms=delay_ms)",
    }
    positions: dict[str, int] = {}
    for key, fragment in fragments.items():
        try:
            positions[key] = text.index(fragment)
        except ValueError:
            fail(f"0902 handler source missing order fragment: {fragment}")

    if not (
        positions["timing"]
        < positions["terrain"]
        < positions["delay"]
        < positions["init_branch"]
        < positions["queue"]
        < positions["schedule"]
    ):
        fail(f"0902 handler timing/terrain/deferred source order changed: {positions!r}")


def make_handler_window(gui: Any, *, init_running: bool) -> Any:
    window = gui.MainWindow.__new__(gui.MainWindow)
    window._power_on = True
    window._initplan_running = bool(init_running)
    window._staged_plan_context = {}
    window._pending_plan_push = object()
    window._attack_delivery_buffer = ["existing"]
    window._events = []
    window._logs = []
    payload = sample_payload()

    window._parse_replan_payload = lambda _raw: payload
    window._append_log_line = lambda text: window._logs.append(str(text))
    window._start_replan_timing = lambda ctx, data: window._events.append(("timing", list(ctx.get("plan_ids") or [])))
    window._schedule_replan_terrain_warmup = (
        lambda ctx, data: window._events.append(("terrain", list(ctx.get("plan_ids") or [])))
    )
    window._replan_delay_ms_for_payload = lambda data: window._events.append(("delay", None)) or 1234
    window._queue_deferred_replan_request = (
        lambda ctx, *, delay_ms: window._events.append(("queue", int(delay_ms), list(ctx.get("plan_ids") or [])))
    )
    window._schedule_replan_pipeline = lambda delay_ms=1000: window._events.append(("schedule", int(delay_ms)))
    window._capture_replan_payload_for_replay = (
        lambda data, ctx: window._events.append(("capture", list(ctx.get("plan_ids") or [])))
    )
    window._to_optional_int = lambda value: int(value) if value is not None else None
    window._is_post_attack_rejoin_detail = lambda _detail: False
    window._prepare_follow_up_attack_detail = lambda _detail: False
    window._log_path_deviation_event = lambda *_args, **_kwargs: None
    window._log_imaging_schedule_event = lambda *_args, **_kwargs: None
    return window


def check_init_running_behavior_order(gui: Any) -> None:
    window = make_handler_window(gui, init_running=True)
    window._handle_replan_received_impl("0902", b"ignored")

    event_names = [event[0] for event in window._events]
    if event_names != ["timing", "terrain", "delay", "queue", "capture"]:
        fail(f"0902 init-running event order changed: {window._events!r}")
    queue_event = window._events[3]
    if queue_event != ("queue", 1234, [700000101, 700000102, 700000103]):
        fail(f"0902 init-running queue payload changed: {queue_event!r}")
    if any(event[0] == "schedule" for event in window._events):
        fail(f"0902 init-running path unexpectedly scheduled immediately: {window._events!r}")


def check_immediate_behavior_order(gui: Any) -> None:
    window = make_handler_window(gui, init_running=False)
    window._handle_replan_received_impl("0902", b"ignored")

    event_names = [event[0] for event in window._events]
    if event_names != ["timing", "terrain", "delay", "schedule", "capture"]:
        fail(f"0902 immediate event order changed: {window._events!r}")
    if window._events[3] != ("schedule", 1234):
        fail(f"0902 immediate schedule delay changed: {window._events[3]!r}")
    if any(event[0] == "queue" for event in window._events):
        fail(f"0902 immediate path unexpectedly queued deferred request: {window._events!r}")


def main() -> int:
    try:
        import importlib

        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_handler_source_order()
        check_init_running_behavior_order(gui)
        check_immediate_behavior_order(gui)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 pre-deferred timing/terrain order smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
