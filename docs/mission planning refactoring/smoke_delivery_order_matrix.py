from __future__ import annotations

import copy
import json
import os
import sys
import time
import types
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("KU_ROLE", "mission")


class SmokeFailure(RuntimeError):
    pass


class SignalSink:
    def __init__(self, callback: Callable[..., Any] | None = None) -> None:
        self.items: list[tuple[Any, ...]] = []
        self._callback = callback

    def emit(self, *args: Any) -> None:
        self.items.append(tuple(args))
        if self._callback is not None:
            self._callback(*args)


class FakeTimer:
    def __init__(self) -> None:
        self.active = False
        self.starts: list[int] = []
        self.stops = 0

    def isActive(self) -> bool:
        return bool(self.active)

    def start(self, delay_ms: int) -> None:
        self.active = True
        self.starts.append(int(delay_ms))

    def stop(self) -> None:
        self.active = False
        self.stops += 1


class FakeItem:
    def __init__(self, text: str) -> None:
        self._text = str(text)

    def text(self) -> str:
        return self._text


class FakeTable:
    def __init__(self, codes: list[str]) -> None:
        self._codes = list(codes)

    def rowCount(self) -> int:
        return len(self._codes)

    def item(self, row: int, column: int) -> FakeItem | None:
        if column != 0 or row < 0 or row >= len(self._codes):
            return None
        return FakeItem(self._codes[row])


class FakeTab:
    def __init__(self) -> None:
        self.tbl_tx = FakeTable(["0301", "0305", "0901", "0903", "0702", "0001"])
        self.sent: list[tuple[str, bytes | None]] = []

    def mark_sent(self, msg_id: str, raw: bytes | None) -> None:
        self.sent.append((str(msg_id), raw))


class FakeSlider:
    def __init__(self, value: int) -> None:
        self._value = int(value)

    def value(self) -> int:
        return int(self._value)


class FakePushCenter:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    def push_message(
        self,
        msg_id: str,
        _messenger: Any,
        *,
        on_done: Callable[[str, bytes], Any] | None = None,
        body_dict: dict[str, Any] | None = None,
    ) -> bool:
        body = copy.deepcopy(body_dict or {})
        self.calls.append({"msg_id": str(msg_id), "body": body})
        raw = json.dumps(body, ensure_ascii=False, sort_keys=True).encode("utf-8")
        if callable(on_done):
            on_done(str(msg_id), raw)
        return True


def fail(message: str) -> None:
    raise SmokeFailure(message)


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


class patched_environment:
    def __init__(self, gui: Any, fake_push: FakePushCenter) -> None:
        self.gui = gui
        self.fake_push = fake_push
        self._old_push_module: Any = None
        self._had_push_module = False
        self._old_single_shot: Any = None
        self.timer_calls: list[int] = []
        self._old_env: dict[str, str | None] = {}

    def __enter__(self) -> "patched_environment":
        module = types.ModuleType("push_center")
        module.push_message = self.fake_push.push_message
        self._had_push_module = "push_center" in sys.modules
        self._old_push_module = sys.modules.get("push_center")
        sys.modules["push_center"] = module

        self._old_single_shot = self.gui.QTimer.singleShot

        def fake_single_shot(delay_ms: int, callback: Callable[[], Any]) -> None:
            self.timer_calls.append(int(delay_ms))
            callback()

        self.gui.QTimer.singleShot = staticmethod(fake_single_shot)

        env_defaults = {
            "REPLAN_OPTION_POST_0301_GRACE_MS": "0",
            "REPLAN_OPTION_POST_0301_PER_EXTRA_PLAN_MS": "0",
            "REPLAN_OPTION_POST_0301_TIMEOUT_EXTRA_MS": "20",
            "REPLAN_FORCE_DIRECT_POST_0301_GRACE_MS": "0",
            "REPLAN_FORCE_DIRECT_POST_0301_TIMEOUT_MS": "1200",
        }
        for key, value in env_defaults.items():
            self._old_env[key] = os.environ.get(key)
            os.environ[key] = value
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _tb: Any) -> None:
        self.gui.QTimer.singleShot = self._old_single_shot
        if self._had_push_module:
            sys.modules["push_center"] = self._old_push_module
        else:
            sys.modules.pop("push_center", None)
        for key, value in self._old_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        cleanup_process_console_state()


def make_window(gui: Any, *, mode_value: int = 3, auto_start: bool = True) -> Any:
    window = gui.MainWindow.__new__(gui.MainWindow)
    window.log_sig = SignalSink()
    window._logs: list[str] = []
    window._timing_events: list[tuple[str, dict[str, Any]]] = []
    window._power_on = True
    window._pending_plan_push = None
    window._scheduled_0301_plan_ids = []
    window._post_0301_delivery = None
    window._post_0301_timer = FakeTimer()
    window._attack_delivery_buffer = []
    window._active_plan_context = {}
    window._option_id_counter = 0
    window._tab = FakeTab()
    window.mode_slider = FakeSlider(mode_value)
    window._orig_mark_sent = None

    window._append_log_line = lambda text: window._logs.append(str(text))
    window._record_replan_timing_event = (
        lambda name, ctx=None, extra=None: window._timing_events.append((str(name), dict(extra or {})))
    )
    window._mark_planning_metric_start = lambda _reason: None
    window._mark_planning_metric_finish = lambda _reason, success=True: 0.0
    window._should_prefix_0305_reason_for_next_collab = lambda: False
    window._consume_attack_delivery_suppress_flag = lambda *, phase: False
    window._flush_deferred_id_tab_update = lambda: window._logs.append("[SMOKE] flush_deferred_id_tab_update")
    window._schedule_post_delivery_waypoint_mark = (
        lambda payload: window._logs.append(f"[SMOKE] waypoint_mark={bool(payload)}")
    )
    window._schedule_post_delivery_snapshot_carry_forward = (
        lambda payload: window._logs.append(f"[SMOKE] snapshot_carry={bool(payload)}")
    )
    callback = window._start_push_sequence if auto_start else None
    window.start_push_seq = SignalSink(callback)
    return window


def msg_ids(fake_push: FakePushCenter) -> list[str]:
    return [str(call["msg_id"]) for call in fake_push.calls]


def bodies(fake_push: FakePushCenter, msg_id: str) -> list[dict[str, Any]]:
    return [dict(call["body"]) for call in fake_push.calls if call["msg_id"] == msg_id]


def check_delivery_helpers() -> None:
    from modules.mission_planning.app.delivery.mission_plan_delivery import (
        post_0301_delivery_delays,
        sort_plan_delivery_entries,
    )

    plan_ids, option_names = sort_plan_delivery_entries(
        [3003, "bad", 3001, 3003, 0, 3002],
        ["third", "bad", "first", "dup", "zero", "second"],
    )
    if plan_ids != [3001, 3002, 3003] or option_names != ["first", "second", "third"]:
        fail(f"delivery plan sort/dedupe changed: {(plan_ids, option_names)!r}")

    option_delays = post_0301_delivery_delays(
        plan_count=3,
        force_direct=False,
        environ={
            "REPLAN_OPTION_POST_0301_GRACE_MS": "10",
            "REPLAN_OPTION_POST_0301_PER_EXTRA_PLAN_MS": "5",
            "REPLAN_OPTION_POST_0301_TIMEOUT_EXTRA_MS": "20",
        },
    )
    if option_delays != (20, 40, "option_or_0901"):
        fail(f"option delivery delay contract changed: {option_delays!r}")

    direct_delays = post_0301_delivery_delays(
        plan_count=1,
        force_direct=True,
        environ={
            "REPLAN_FORCE_DIRECT_POST_0301_GRACE_MS": "99",
            "REPLAN_FORCE_DIRECT_POST_0301_TIMEOUT_MS": "50",
        },
    )
    if direct_delays != (99, 199, "direct_0903"):
        fail(f"direct delivery timeout clamp changed: {direct_delays!r}")


def check_pending_delivery_merge(gui: Any) -> None:
    window = make_window(gui, auto_start=False)
    window._schedule_plan_delivery(
        [3003, 3001],
        ["third", "first"],
        "merge smoke",
        option_meta={3003: {"label": "three"}},
    )
    window._schedule_plan_delivery(
        [3002, 3001],
        ["second", "duplicate"],
        "merge smoke",
        option_meta={3002: {"label": "two"}},
    )

    pending = dict(window._pending_plan_push or {})
    if pending.get("plan_ids") != [3001, 3002, 3003]:
        fail(f"pending delivery merge plan order changed: {pending!r}")
    if pending.get("option_names") != ["first", "second", "third"]:
        fail(f"pending delivery merge option names changed: {pending!r}")
    meta = pending.get("option_meta")
    if not isinstance(meta, dict) or sorted(meta) != [3002, 3003]:
        fail(f"pending delivery meta merge changed: {pending!r}")
    if [name for name, _extra in window._timing_events].count("0301_merged") != 1:
        fail(f"pending delivery merge timing event changed: {window._timing_events!r}")


def check_option_mode_order(gui: Any) -> None:
    fake_push = FakePushCenter()
    with patched_environment(gui, fake_push):
        window = make_window(gui, mode_value=3, auto_start=True)
        window._schedule_plan_delivery(
            [3102, 3101],
            ["option2", "option1"],
            "option mode smoke",
        )

        if msg_ids(fake_push) != ["0301", "0301", "0305"]:
            fail(f"option mode should wait for mode_ready before 0901: {msg_ids(fake_push)!r}")
        pending = dict(window._post_0301_delivery or {})
        if not pending or not pending.get("completion_ready") or pending.get("mode_ready"):
            fail(f"option mode pending readiness changed before mode_ready: {pending!r}")

        flushed = window._mark_post_0301_ready(trigger="smoke_mode_ready")
        if not flushed:
            fail("option mode did not flush after mode_ready")
        if msg_ids(fake_push) != ["0301", "0301", "0305", "0901"]:
            fail(f"option mode delivery order changed: {msg_ids(fake_push)!r}")

        plan_ids_0301 = [body["missionPlanID"] for body in bodies(fake_push, "0301")]
        if plan_ids_0301 != [3101, 3102]:
            fail(f"0301 sorted plan order changed: {plan_ids_0301!r}")
        body_0901 = bodies(fake_push, "0901")[0]
        option_plan_ids = [entry["missionPlanID"] for entry in body_0901["pendingOptionList"]]
        if option_plan_ids != [3101, 3102]:
            fail(f"0901 pending option plan order changed: {body_0901!r}")
        if window._post_0301_delivery is not None:
            fail("option mode did not clear post-0301 delivery after flush")


def check_apply_mode_order(gui: Any) -> None:
    fake_push = FakePushCenter()
    with patched_environment(gui, fake_push) as patch:
        window = make_window(gui, mode_value=0, auto_start=True)
        window._schedule_plan_delivery(
            [3202, 3201],
            ["option2", "option1"],
            "apply mode smoke",
        )

        if msg_ids(fake_push) != ["0301", "0301", "0305"]:
            fail(f"apply mode should wait for mode_ready before 0903: {msg_ids(fake_push)!r}")
        if not window._mark_post_0301_ready(trigger="smoke_mode_ready"):
            fail("apply mode did not flush after mode_ready")
        if msg_ids(fake_push) != ["0301", "0301", "0305", "0903", "0903"]:
            fail(f"apply mode delivery order changed: {msg_ids(fake_push)!r}")
        if patch.timer_calls != [0, 200]:
            fail(f"apply mode 0903 timer spacing changed: {patch.timer_calls!r}")
        plan_ids_0903 = [body["missionPlanID"] for body in bodies(fake_push, "0903")]
        if plan_ids_0903 != [3201, 3202]:
            fail(f"0903 sorted plan order changed: {plan_ids_0903!r}")
        if "0702" in msg_ids(fake_push):
            fail(f"non-direct apply mode unexpectedly sent 0702: {msg_ids(fake_push)!r}")


def check_force_direct_basic_order(gui: Any) -> None:
    fake_push = FakePushCenter()
    with patched_environment(gui, fake_push) as patch:
        window = make_window(gui, mode_value=3, auto_start=True)
        window._schedule_plan_delivery(
            [3301],
            ["option1"],
            "force direct smoke",
            force_direct_update=True,
            suppress_0702_fallback=False,
        )

        if msg_ids(fake_push) != ["0301", "0305", "0903", "0702"]:
            fail(f"force-direct basic delivery order changed: {msg_ids(fake_push)!r}")
        if patch.timer_calls != [0, 250]:
            fail(f"force-direct 0903/0702 timer spacing changed: {patch.timer_calls!r}")
        body_0702 = bodies(fake_push, "0702")[0]
        if body_0702.get("ignore") != 2 or body_0702.get("missionPlanID") != 3301:
            fail(f"0702 auto-apply payload changed: {body_0702!r}")


def check_force_direct_suppressed_0702_order(gui: Any) -> None:
    fake_push = FakePushCenter()
    with patched_environment(gui, fake_push) as patch:
        window = make_window(gui, mode_value=3, auto_start=True)
        window._schedule_plan_delivery(
            [3311],
            ["option1"],
            "force direct suppressed smoke",
            force_direct_update=True,
            suppress_0702_fallback=True,
        )

        if msg_ids(fake_push) != ["0301", "0305", "0903"]:
            fail(f"force-direct suppressed delivery order changed: {msg_ids(fake_push)!r}")
        if patch.timer_calls != [0]:
            fail(f"force-direct suppressed timer spacing changed: {patch.timer_calls!r}")
        if not any(name == "0702_suppressed" for name, _extra in window._timing_events):
            fail(f"force-direct suppressed timing event changed: {window._timing_events!r}")


def check_0301_missing_plan_blocks_delivery(gui: Any) -> None:
    fake_push = FakePushCenter()
    with patched_environment(gui, fake_push):
        window = make_window(gui, mode_value=3, auto_start=True)
        window._pending_plan_push = {
            "plan_ids": [],
            "option_names": [],
            "reason": "missing plan smoke",
            "option_meta": {},
            "force_direct_update": False,
            "suppress_0702_fallback": False,
        }
        window._start_push_sequence()
        if fake_push.calls:
            fail(f"missing plan should not push any messages: {fake_push.calls!r}")
        if window._pending_plan_push is None:
            fail("missing plan currently leaves pending payload intact; contract changed")
        if not any("No missionPlanID" in line for line in window._logs):
            fail(f"missing plan warning changed: {window._logs!r}")


def main() -> int:
    try:
        import importlib

        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_delivery_helpers()
        check_pending_delivery_merge(gui)
        check_option_mode_order(gui)
        check_apply_mode_order(gui)
        check_force_direct_basic_order(gui)
        check_force_direct_suppressed_0702_order(gui)
        check_0301_missing_plan_blocks_delivery(gui)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        cleanup_process_console_state()

    print("delivery order matrix smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
