from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def load_delivery_smoke() -> Any:
    path = Path(__file__).with_name("smoke_delivery_order_matrix.py")
    spec = importlib.util.spec_from_file_location("delivery_order_matrix_smoke_helpers_for_post_delivery", path)
    if spec is None or spec.loader is None:
        fail(f"cannot load delivery smoke helpers from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ImmediateThread:
    started: list[str] = []

    def __init__(
        self,
        *,
        target: Callable[[], Any],
        name: str | None = None,
        daemon: bool | None = None,
    ) -> None:
        self._target = target
        self.name = name
        self.daemon = daemon

    def start(self) -> None:
        ImmediateThread.started.append(str(self.name or ""))
        self._target()


class patched_workers:
    def __init__(self, gui: Any) -> None:
        self.gui = gui
        self.waypoint_calls: list[int | None] = []
        self.snapshot_calls: list[tuple[int, int, str]] = []

    def __enter__(self) -> "patched_workers":
        self.original_thread = self.gui.threading.Thread
        self.gui.threading.Thread = ImmediateThread
        ImmediateThread.started.clear()

        self.allocator = importlib.import_module(
            "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
        )
        self.original_mark = self.allocator.mark_waypoint_files_written
        self.allocator.mark_waypoint_files_written = self._mark_waypoints

        self.original_carry = self.gui.mission_area_replan_store.carry_forward_snapshot
        self.gui.mission_area_replan_store.carry_forward_snapshot = self._carry_snapshot
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _tb: Any) -> None:
        self.gui.threading.Thread = self.original_thread
        self.allocator.mark_waypoint_files_written = self.original_mark
        self.gui.mission_area_replan_store.carry_forward_snapshot = self.original_carry

    def _mark_waypoints(self, max_waypoint_id: int | None = None) -> None:
        self.waypoint_calls.append(max_waypoint_id)

    def _carry_snapshot(self, source_plan_id: int, target_plan_id: int, *, reason: str = "") -> Path | None:
        self.snapshot_calls.append((int(source_plan_id), int(target_plan_id), str(reason)))
        if int(target_plan_id) == 6202:
            return None
        return Path(f"carried_{source_plan_id}_{target_plan_id}.json")


def event_extra(window: Any, name: str) -> list[dict[str, Any]]:
    return [dict(extra) for event_name, extra in window._timing_events if event_name == name]


def log_text(window: Any) -> str:
    return "\n".join(["\n".join(window._logs), *(" ".join(str(part) for part in item) for item in window.log_sig.items)])


def check_helper_normalization_and_merge(gui: Any) -> None:
    mark = gui.MainWindow._normalize_post_delivery_waypoint_mark(
        {"max_waypoint_id": "42", "variants": "-3"}
    )
    if mark != {"max_waypoint_id": 42, "variants": 0, "reason": "post_delivery_waypoint_mark"}:
        fail(f"waypoint mark normalization changed: {mark!r}")
    if gui.MainWindow._normalize_post_delivery_waypoint_mark({"max_waypoint_id": 0, "variants": 0}) is not None:
        fail("empty waypoint mark payload is no longer ignored")
    merged_mark = gui.MainWindow._merge_post_delivery_waypoint_mark(
        {"max_waypoint_id": 10, "variants": 2, "reason": "old"},
        {"max_waypoint_id": 7, "variants": 3, "reason": "new"},
    )
    if merged_mark != {"max_waypoint_id": 10, "variants": 5, "reason": "new"}:
        fail(f"waypoint mark merge changed: {merged_mark!r}")

    snapshot = gui.MainWindow._normalize_post_delivery_snapshot_carry_forward(
        {
            "items": [
                {"source_plan_id": "6101", "target_plan_id": "6201", "variant_no": "-2"},
                {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6202, "variant": 3, "reason": "item"},
                {"sourceMissionPlanID": 0, "targetMissionPlanID": 6203},
                "bad",
            ],
            "reason": "batch",
        }
    )
    expected_snapshot = {
        "items": [
            {
                "sourceMissionPlanID": 6101,
                "targetMissionPlanID": 6201,
                "variant": 0,
                "reason": "batch",
            },
            {
                "sourceMissionPlanID": 6101,
                "targetMissionPlanID": 6202,
                "variant": 3,
                "reason": "item",
            },
        ],
        "reason": "batch",
    }
    if snapshot != expected_snapshot:
        fail(f"snapshot carry normalization changed: {snapshot!r}")
    if gui.MainWindow._normalize_post_delivery_snapshot_carry_forward({"items": "bad"}) is not None:
        fail("string snapshot items are no longer ignored")

    merged_snapshot = gui.MainWindow._merge_post_delivery_snapshot_carry_forward(
        {
            "items": [
                {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "reason": "same"},
                {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "reason": "same"},
            ],
            "reason": "old",
        },
        {
            "items": [
                {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "reason": "new"},
                {"sourceMissionPlanID": 3, "targetMissionPlanID": 4, "reason": "same"},
            ],
            "reason": "new-batch",
        },
    )
    expected_merged_items = [
        {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "variant": 0, "reason": "same"},
        {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "variant": 0, "reason": "new"},
        {"sourceMissionPlanID": 3, "targetMissionPlanID": 4, "variant": 0, "reason": "same"},
    ]
    if not merged_snapshot or merged_snapshot.get("items") != expected_merged_items:
        fail(f"snapshot merge/dedupe changed: {merged_snapshot!r}")
    if merged_snapshot.get("reason") != "new-batch":
        fail(f"snapshot merge reason changed: {merged_snapshot!r}")


def check_schedule_plan_delivery_carries_post_payloads(gui: Any, helpers: Any) -> None:
    window = helpers.make_window(gui, auto_start=False)
    window._schedule_plan_delivery(
        [7102],
        ["option2"],
        "post delivery merge",
        post_delivery_waypoint_mark={"max_waypoint_id": 10, "variants": 1, "reason": "old"},
        post_delivery_snapshot_carry_forward={
            "items": [{"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "reason": "same"}],
            "reason": "old",
        },
    )
    window._schedule_plan_delivery(
        [7101],
        ["option1"],
        "post delivery merge",
        post_delivery_waypoint_mark={"max_waypoint_id": 12, "variants": 2, "reason": "new"},
        post_delivery_snapshot_carry_forward={
            "items": [
                {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "reason": "same"},
                {"sourceMissionPlanID": 3, "targetMissionPlanID": 4, "reason": "new"},
            ],
            "reason": "new",
        },
    )
    pending = dict(window._pending_plan_push or {})
    if pending.get("plan_ids") != [7101, 7102]:
        fail(f"post delivery pending plan merge changed: {pending!r}")
    if pending.get("post_delivery_waypoint_mark") != {
        "max_waypoint_id": 12,
        "variants": 3,
        "reason": "new",
    }:
        fail(f"pending waypoint mark merge changed: {pending!r}")
    carry = pending.get("post_delivery_snapshot_carry_forward")
    if not isinstance(carry, dict) or carry.get("reason") != "new":
        fail(f"pending snapshot carry merge reason changed: {pending!r}")
    if carry.get("items") != [
        {"sourceMissionPlanID": 1, "targetMissionPlanID": 2, "variant": 0, "reason": "same"},
        {"sourceMissionPlanID": 3, "targetMissionPlanID": 4, "variant": 0, "reason": "new"},
    ]:
        fail(f"pending snapshot carry merge items changed: {pending!r}")


def check_start_push_sequence_schedules_only_after_success(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with helpers.patched_environment(gui, fake_push):
        window = helpers.make_window(gui, mode_value=3, auto_start=False)
        waypoint_calls: list[Any] = []
        snapshot_calls: list[Any] = []
        window._schedule_post_delivery_waypoint_mark = lambda payload: waypoint_calls.append(payload)
        window._schedule_post_delivery_snapshot_carry_forward = lambda payload: snapshot_calls.append(payload)
        mark_payload = {"max_waypoint_id": 20, "variants": 2}
        snapshot_payload = {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6201}
        window._pending_plan_push = {
            "plan_ids": [7201],
            "option_names": ["option1"],
            "reason": "post delivery success",
            "option_meta": {},
            "force_direct_update": False,
            "suppress_0702_fallback": False,
            "post_delivery_waypoint_mark": mark_payload,
            "post_delivery_snapshot_carry_forward": snapshot_payload,
        }
        window._scheduled_0301_plan_ids = [7201]
        window._start_push_sequence()

        if helpers.msg_ids(fake_push) != ["0301", "0305"]:
            fail(f"post delivery success setup should send 0301/0305: {helpers.msg_ids(fake_push)!r}")
        if waypoint_calls != [mark_payload] or snapshot_calls != [snapshot_payload]:
            fail(
                "post delivery payloads should be scheduled only after 0301/0305 success: "
                f"waypoint={waypoint_calls!r}, snapshot={snapshot_calls!r}"
            )
        pending_after_completion = window._post_0301_delivery
        if not isinstance(pending_after_completion, dict) or not pending_after_completion.get("completion_ready"):
            fail(
                "post delivery scheduling should occur while post-0301 delivery is still waiting for mode-ready: "
                f"{pending_after_completion!r}"
            )
        if pending_after_completion.get("mode_ready"):
            fail(f"post delivery scheduling should precede mode-ready flush: {pending_after_completion!r}")

    fake_push = helpers.FakePushCenter()
    with helpers.patched_environment(gui, fake_push):
        window = helpers.make_window(gui, mode_value=3, auto_start=False)
        waypoint_calls = []
        snapshot_calls = []
        window._schedule_post_delivery_waypoint_mark = lambda payload: waypoint_calls.append(payload)
        window._schedule_post_delivery_snapshot_carry_forward = lambda payload: snapshot_calls.append(payload)
        window._click_tx_button_for = lambda _code: False
        window._pending_plan_push = {
            "plan_ids": [7202],
            "option_names": ["option1"],
            "reason": "post delivery 0301 failure",
            "option_meta": {},
            "post_delivery_waypoint_mark": {"max_waypoint_id": 20},
            "post_delivery_snapshot_carry_forward": {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6201},
        }
        window._start_push_sequence()
        if fake_push.calls or waypoint_calls or snapshot_calls:
            fail(
                "post delivery should not schedule after 0301 failure: "
                f"push={fake_push.calls!r}, waypoint={waypoint_calls!r}, snapshot={snapshot_calls!r}"
            )

    fake_push = helpers.FakePushCenter()
    with helpers.patched_environment(gui, fake_push):
        window = helpers.make_window(gui, mode_value=3, auto_start=False)
        waypoint_calls = []
        snapshot_calls = []
        window._schedule_post_delivery_waypoint_mark = lambda payload: waypoint_calls.append(payload)
        window._schedule_post_delivery_snapshot_carry_forward = lambda payload: snapshot_calls.append(payload)
        window._push_post_0301_completion = lambda *, reason: False
        window._pending_plan_push = {
            "plan_ids": [7203],
            "option_names": ["option1"],
            "reason": "post delivery completion failure",
            "option_meta": {},
            "post_delivery_waypoint_mark": {"max_waypoint_id": 20},
            "post_delivery_snapshot_carry_forward": {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6201},
        }
        window._scheduled_0301_plan_ids = [7203]
        window._start_push_sequence()
        if helpers.msg_ids(fake_push) != ["0301"]:
            fail(f"0305 failure setup should only send 0301: {helpers.msg_ids(fake_push)!r}")
        if waypoint_calls or snapshot_calls:
            fail(
                "post delivery should not schedule after 0305 failure: "
                f"waypoint={waypoint_calls!r}, snapshot={snapshot_calls!r}"
            )


def check_actual_schedule_workers(gui: Any, helpers: Any) -> None:
    with patched_workers(gui) as workers:
        window = helpers.make_window(gui, auto_start=False)
        window._schedule_post_delivery_waypoint_mark = (
            gui.MainWindow._schedule_post_delivery_waypoint_mark.__get__(window, gui.MainWindow)
        )
        window._schedule_post_delivery_snapshot_carry_forward = (
            gui.MainWindow._schedule_post_delivery_snapshot_carry_forward.__get__(window, gui.MainWindow)
        )

        window._schedule_post_delivery_waypoint_mark({"max_waypoint_id": 99, "variants": 4})
        if workers.waypoint_calls != [99]:
            fail(f"waypoint worker mark call changed: {workers.waypoint_calls!r}")
        waypoint_events = event_extra(window, "post_delivery_waypoint_mark")
        if not waypoint_events:
            fail(f"waypoint worker timing event missing: {window._timing_events!r}")
        waypoint_extra = waypoint_events[-1]
        for key, value in {"max_waypoint_id": 99, "variants": 4, "outcome": "ok"}.items():
            if waypoint_extra.get(key) != value:
                fail(f"waypoint worker timing {key} changed: {waypoint_extra!r}")
        if "PostDelivery-WaypointMark" not in ImmediateThread.started:
            fail(f"waypoint worker thread name changed: {ImmediateThread.started!r}")

        window._schedule_post_delivery_waypoint_mark({"max_waypoint_id": 0, "variants": 0})
        if workers.waypoint_calls != [99]:
            fail("invalid waypoint payload should not start worker")

        window._schedule_post_delivery_snapshot_carry_forward(
            {
                "items": [
                    {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6201, "reason": "ok"},
                    {"sourceMissionPlanID": 6101, "targetMissionPlanID": 6202, "reason": "skip"},
                ],
                "reason": "batch",
            }
        )
        if workers.snapshot_calls != [
            (6101, 6201, "ok"),
            (6101, 6202, "skip"),
        ]:
            fail(f"snapshot worker carry calls changed: {workers.snapshot_calls!r}")
        snapshot_events = event_extra(window, "post_delivery_snapshot_carry_forward")
        if not snapshot_events:
            fail(f"snapshot worker timing event missing: {window._timing_events!r}")
        snapshot_extra = snapshot_events[-1]
        expected = {"items": 2, "carried": 1, "skipped": 1, "errors": 0, "outcome": "ok"}
        for key, value in expected.items():
            if snapshot_extra.get(key) != value:
                fail(f"snapshot worker timing {key} changed: {snapshot_extra!r}")
        if "PostDelivery-SnapshotCarry" not in ImmediateThread.started:
            fail(f"snapshot worker thread name changed: {ImmediateThread.started!r}")

        window._schedule_post_delivery_snapshot_carry_forward({"items": "bad"})
        if len(workers.snapshot_calls) != 2:
            fail("invalid snapshot payload should not start worker")


def main() -> int:
    helpers = load_delivery_smoke()
    try:
        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_helper_normalization_and_merge(gui)
        check_schedule_plan_delivery_carries_post_payloads(gui, helpers)
        check_start_push_sequence_schedules_only_after_success(gui, helpers)
        check_actual_schedule_workers(gui, helpers)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        helpers.cleanup_process_console_state()

    print("post delivery carry-forward smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
