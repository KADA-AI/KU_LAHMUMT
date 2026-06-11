from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

QUALITY_REASON = "\ucd2c\uc601 \ud488\uc9c8 \uac1c\uc120 request"
QUALITY_TRIGGER = "qualityMonitorSep"


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def load_delivery_smoke() -> Any:
    path = Path(__file__).with_name("smoke_delivery_order_matrix.py")
    spec = importlib.util.spec_from_file_location("delivery_order_matrix_smoke_helpers", path)
    if spec is None or spec.loader is None:
        fail(f"cannot load delivery smoke helpers from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class patched_suppress_reader:
    def __enter__(self) -> "patched_suppress_reader":
        self.module = importlib.import_module("modules.monitoring.logic.replan_queue_manager")
        self.original = self.module.read_and_clear_suppress_option_flag
        self.module.read_and_clear_suppress_option_flag = lambda: None
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _tb: Any) -> None:
        self.module.read_and_clear_suppress_option_flag = self.original


def quality_meta(plan_id: int, *, nested: bool = False) -> dict[int, dict[str, Any]]:
    if nested:
        return {int(plan_id): {"replanDetail": {"triggerType": QUALITY_TRIGGER}}}
    return {int(plan_id): {"triggerType": QUALITY_TRIGGER}}


def event_extra(window: Any, name: str) -> list[dict[str, Any]]:
    return [dict(extra) for event_name, extra in window._timing_events if event_name == name]


def assert_ids(actual: list[str], expected: list[str], label: str) -> None:
    if actual != expected:
        fail(f"{label} order changed: actual={actual!r}, expected={expected!r}")


def run_delivery(
    gui: Any,
    helpers: Any,
    action: Callable[[Any], None],
) -> tuple[Any, Any, Any]:
    fake_push = helpers.FakePushCenter()
    with helpers.patched_environment(gui, fake_push) as patch, patched_suppress_reader():
        window = helpers.make_window(gui, mode_value=3, auto_start=True)
        action(window)
        return window, fake_push, patch


def check_quality_predicates(gui: Any) -> None:
    if not gui._is_quality_speed_reason_text(QUALITY_REASON):
        fail("quality-speed reason predicate no longer recognizes the Korean quality reason")
    if gui._is_quality_speed_reason_text("\ucd2c\uc601\ud488\uc9c8\uac1c\uc120 request"):
        fail("quality-speed reason predicate no longer requires the current exact keyword text")
    if not gui._is_quality_speed_trigger_type(QUALITY_TRIGGER):
        fail("quality-speed trigger predicate changed")
    if gui._is_quality_speed_trigger_type("QualityMonitorSep"):
        fail("quality-speed trigger predicate is no longer exact-case")
    if not gui._plan_meta_has_quality_speed(quality_meta(4101)):
        fail("quality-speed plan_meta triggerType detection changed")
    if not gui._plan_meta_has_quality_speed(quality_meta(4101, nested=True)):
        fail("quality-speed nested replanDetail triggerType detection changed")
    if gui._plan_meta_has_quality_speed({4101: {"triggerType": "imagingScheduleDeviation"}}):
        fail("non-quality imaging meta is now treated as quality-speed")


def check_quality_reason_forces_direct(gui: Any, helpers: Any) -> None:
    window, fake_push, patch = run_delivery(
        gui,
        helpers,
        lambda w: w._schedule_plan_delivery(
            [4101],
            ["option1"],
            QUALITY_REASON,
        ),
    )
    assert_ids(helpers.msg_ids(fake_push), ["0301", "0305", "0903"], "quality reason delivery")
    if patch.timer_calls != [0]:
        fail(f"quality reason should only schedule immediate 0903: {patch.timer_calls!r}")
    pipeline_done = event_extra(window, "pipeline_done")
    if not pipeline_done:
        fail(f"quality reason missing pipeline_done event: {window._timing_events!r}")
    extra = pipeline_done[-1]
    expected = {
        "force_direct": True,
        "suppress_0702": True,
        "quality_speed": True,
        "option_names": 1,
    }
    for key, value in expected.items():
        if extra.get(key) != value:
            fail(f"quality reason pipeline_done {key} changed: {extra!r}")
    if any(name == "0901" or name == "0702" for name in helpers.msg_ids(fake_push)):
        fail(f"quality reason should suppress 0901/0702: {helpers.msg_ids(fake_push)!r}")
    if not any(name == "0702_suppressed" for name, _extra in window._timing_events):
        fail(f"quality reason did not record 0702_suppressed: {window._timing_events!r}")


def check_quality_meta_forces_direct(gui: Any, helpers: Any) -> None:
    window, fake_push, patch = run_delivery(
        gui,
        helpers,
        lambda w: w._schedule_plan_delivery(
            [4201],
            ["option1"],
            "normal reason",
            option_meta=quality_meta(4201, nested=True),
        ),
    )
    assert_ids(helpers.msg_ids(fake_push), ["0301", "0305", "0903"], "quality meta delivery")
    if patch.timer_calls != [0]:
        fail(f"quality meta should only schedule immediate 0903: {patch.timer_calls!r}")
    pipeline_done = event_extra(window, "pipeline_done")
    if not pipeline_done or not pipeline_done[-1].get("quality_speed"):
        fail(f"quality meta did not mark quality_speed: {window._timing_events!r}")


def check_0901_quality_blocks(gui: Any, helpers: Any) -> None:
    cases: list[tuple[str, dict[str, Any], dict[int, dict[str, Any]]]] = [
        ("plan_meta", {}, quality_meta(4301)),
        ("active_reason", {"reason": QUALITY_REASON}, {}),
        ("active_trigger", {"replan_detail": {"triggerType": QUALITY_TRIGGER}}, {}),
    ]
    for label, active_ctx, plan_meta in cases:
        fake_push = helpers.FakePushCenter()
        with helpers.patched_environment(gui, fake_push), patched_suppress_reader():
            window = helpers.make_window(gui, auto_start=False)
            window._active_plan_context = dict(active_ctx)
            window._push_0901_options([4301], ["option1"], plan_meta)
            if fake_push.calls:
                fail(f"quality {label} should block 0901 push: {fake_push.calls!r}")
            logs = "\n".join(" ".join(str(part) for part in item) for item in window.log_sig.items)
            if "0901 blocked" not in logs:
                fail(f"quality {label} 0901 block log changed: {logs!r}")


def check_direct_helper_matrix(gui: Any, helpers: Any) -> None:
    cases: list[tuple[str, Callable[[Any], None], list[str], list[int]]] = [
        (
            "imaging_unsuppressed",
            lambda w: w._deliver_imaging_schedule_direct_now(
                [4401],
                "imaging direct",
                option_names=["option1"],
                suppress_0702_fallback=False,
            ),
            ["0301", "0305", "0903", "0702"],
            [0, 250],
        ),
        (
            "imaging_quality_suppressed",
            lambda w: w._deliver_imaging_schedule_direct_now(
                [4402],
                QUALITY_REASON,
                option_names=["option1"],
                suppress_0702_fallback=True,
            ),
            ["0301", "0305", "0903"],
            [0],
        ),
        (
            "next_collab_direct",
            lambda w: w._deliver_next_collab_direct_now(
                [4403],
                "next collab direct",
                option_names=["option1"],
            ),
            ["0301", "0305", "0903"],
            [0],
        ),
        (
            "path_deviation_direct",
            lambda w: w._deliver_path_deviation_direct_now(
                [4404],
                "path deviation direct",
                option_names=["option1"],
            ),
            ["0301", "0305", "0903", "0702"],
            [0, 250],
        ),
        (
            "prior_direct",
            lambda w: w._deliver_prior_direct_now(
                [4405],
                "prior direct",
                option_names=["option1"],
            ),
            ["0301", "0305", "0903", "0702"],
            [0, 250],
        ),
    ]
    for label, action, expected_ids, expected_timers in cases:
        window, fake_push, patch = run_delivery(gui, helpers, action)
        assert_ids(helpers.msg_ids(fake_push), expected_ids, label)
        if patch.timer_calls != expected_timers:
            fail(f"{label} timer matrix changed: {patch.timer_calls!r}")
        suppressed_events = [extra for name, extra in window._timing_events if name == "0702_suppressed"]
        if "0702" in expected_ids and suppressed_events:
            fail(f"{label} unexpectedly recorded 0702_suppressed: {window._timing_events!r}")
        if "0702" not in expected_ids and not suppressed_events:
            fail(f"{label} missing 0702_suppressed event: {window._timing_events!r}")


def check_late_quality_pending_flush(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with helpers.patched_environment(gui, fake_push) as patch, patched_suppress_reader():
        window = helpers.make_window(gui, mode_value=3, auto_start=False)
        window._queue_post_0301_delivery(
            plan_ids=[4501],
            option_names=["option1"],
            plan_meta=quality_meta(4501),
            is_execution_mode=True,
            force_direct=False,
            suppress_0702_fallback=False,
        )
        pending = window._post_0301_delivery
        if not isinstance(pending, dict):
            fail("late quality pending delivery was not queued")
        pending["completion_ready"] = True

        flushed = window._mark_post_0301_ready(trigger="late_quality")
        if not flushed:
            fail("late quality pending delivery did not flush")
        assert_ids(helpers.msg_ids(fake_push), ["0903"], "late quality pending flush")
        if patch.timer_calls != [0]:
            fail(f"late quality pending flush timer changed: {patch.timer_calls!r}")
        if not any(name == "0702_suppressed" for name, _extra in window._timing_events):
            fail(f"late quality pending flush did not record 0702_suppressed: {window._timing_events!r}")


def check_multi_plan_direct_keeps_0702_fallback(gui: Any, helpers: Any) -> None:
    window, fake_push, patch = run_delivery(
        gui,
        helpers,
        lambda w: w._deliver_prior_direct_now(
            [4602, 4601],
            "prior direct multi",
            option_names=["option2", "option1"],
        ),
    )
    assert_ids(
        helpers.msg_ids(fake_push),
        ["0301", "0301", "0305", "0903", "0702", "0903", "0702"],
        "multi-plan direct fallback",
    )
    if patch.timer_calls != [0, 250, 200, 450]:
        fail(f"multi-plan direct fallback timer schedule changed: {patch.timer_calls!r}")
    plan_ids_0301 = [body["missionPlanID"] for body in helpers.bodies(fake_push, "0301")]
    plan_ids_0903 = [body["missionPlanID"] for body in helpers.bodies(fake_push, "0903")]
    plan_ids_0702 = [body["missionPlanID"] for body in helpers.bodies(fake_push, "0702")]
    if plan_ids_0301 != [4601, 4602] or plan_ids_0903 != [4601, 4602] or plan_ids_0702 != [4601, 4602]:
        fail(
            "multi-plan direct fallback sorted plan order changed: "
            f"0301={plan_ids_0301!r}, 0903={plan_ids_0903!r}, 0702={plan_ids_0702!r}"
        )
    if any(name == "0702_suppressed" for name, _extra in window._timing_events):
        fail(f"multi-plan direct fallback unexpectedly suppressed 0702: {window._timing_events!r}")


def main() -> int:
    helpers = load_delivery_smoke()
    try:
        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_quality_predicates(gui)
        check_quality_reason_forces_direct(gui, helpers)
        check_quality_meta_forces_direct(gui, helpers)
        check_0901_quality_blocks(gui, helpers)
        check_direct_helper_matrix(gui, helpers)
        check_late_quality_pending_flush(gui, helpers)
        check_multi_plan_direct_keeps_0702_fallback(gui, helpers)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        helpers.cleanup_process_console_state()

    print("quality direct delivery suppression smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
