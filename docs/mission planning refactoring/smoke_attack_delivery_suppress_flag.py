from __future__ import annotations

import importlib
import importlib.util
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

SUPPRESS_REASON = "attack suppress smoke reason"


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def load_delivery_smoke() -> Any:
    path = Path(__file__).with_name("smoke_delivery_order_matrix.py")
    spec = importlib.util.spec_from_file_location("delivery_order_matrix_smoke_helpers_for_attack", path)
    if spec is None or spec.loader is None:
        fail(f"cannot load delivery smoke helpers from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


class temp_db:
    def __enter__(self) -> Path:
        from modules.common import db_paths

        self.db_paths = db_paths
        self.original_get_db_subpath = db_paths.get_db_subpath
        self.root = Path(tempfile.mkdtemp(prefix="mp_attack_suppress_"))

        def fake_get_db_subpath(*parts: object) -> Path:
            return self.root.joinpath(*(str(part) for part in parts))

        db_paths.get_db_subpath = fake_get_db_subpath
        return self.root

    def __exit__(self, _exc_type: Any, _exc: Any, _tb: Any) -> None:
        cleanup_process_console_state()
        self.db_paths.get_db_subpath = self.original_get_db_subpath
        shutil.rmtree(self.root, ignore_errors=True)


def flag_path() -> Path:
    from modules.common import db_paths

    return db_paths.get_db_subpath("DSS_Internal", "suppress_option_request.json")


def write_flag(
    *,
    plan_ids: list[int] | None = None,
    target_id: int | None = 71,
    target_key: str | None = "T-71",
    reason: str = SUPPRESS_REASON,
) -> Path:
    path = flag_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "active": True,
        "target_id": target_id,
        "target_key": target_key,
        "queue_id": 7,
        "signature": "attack-suppress-smoke",
        "plan_ids": list(plan_ids or [5101]),
        "created_ms": 123456,
        "reason": str(reason),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def attack_context(plan_id: int = 5101, *, target_id: int = 71, target_key: str = "T-71") -> dict[str, Any]:
    return {
        "plan_ids": [int(plan_id)],
        "reason": "attack delivery smoke",
        "replan_detail": {
            "trigger": "0402",
            "targetID": int(target_id),
            "targetKey": str(target_key),
        },
    }


def non_attack_context(plan_id: int = 5101) -> dict[str, Any]:
    return {
        "plan_ids": [int(plan_id)],
        "reason": "non attack smoke",
        "replan_detail": {
            "trigger": "0401",
            "targetID": 71,
            "targetKey": "T-71",
        },
    }


def pending_payload(plan_id: int = 5101) -> dict[str, Any]:
    return {
        "plan_ids": [int(plan_id)],
        "option_names": ["option1"],
        "reason": "attack delivery smoke",
        "option_meta": {},
        "force_direct_update": False,
        "suppress_0702_fallback": False,
    }


def make_window(gui: Any, helpers: Any, *, active_ctx: dict[str, Any] | None = None) -> Any:
    window = helpers.make_window(gui, mode_value=3, auto_start=False)
    window._active_plan_context = dict(active_ctx or attack_context())
    window._consume_attack_delivery_suppress_flag = (
        gui.MainWindow._consume_attack_delivery_suppress_flag.__get__(window, gui.MainWindow)
    )
    return window


def msg_ids(helpers: Any, fake_push: Any) -> list[str]:
    return helpers.msg_ids(fake_push)


def notice_contents(helpers: Any, fake_push: Any) -> list[str]:
    return [str(body.get("contents") or "") for body in helpers.bodies(fake_push, "0001")]


def log_text(window: Any) -> str:
    log_sig_text = "\n".join(" ".join(str(part) for part in item) for item in window.log_sig.items)
    return "\n".join([*window._logs, log_sig_text])


def check_non_attack_context_does_not_consume(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers, active_ctx=non_attack_context())
        if window._consume_attack_delivery_suppress_flag(phase="0301"):
            fail("non-attack context unexpectedly consumed attack suppress flag")
        if msg_ids(helpers, fake_push):
            fail(f"non-attack context unexpectedly pushed messages: {fake_push.calls!r}")
        if not flag_path().exists():
            fail("non-attack context should not read/clear suppress flag")


def check_matching_0301_suppresses_full_delivery(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers)
        window._pending_plan_push = pending_payload(5101)
        window._scheduled_0301_plan_ids = [5101]
        window._post_0301_delivery = {"old": True}
        window._post_0301_timer.active = True

        window._start_push_sequence()

        if msg_ids(helpers, fake_push) != ["0001"]:
            fail(f"matching 0301 suppress should only push 0001: {msg_ids(helpers, fake_push)!r}")
        if SUPPRESS_REASON not in notice_contents(helpers, fake_push)[0]:
            fail(f"matching 0301 suppress reason changed: {notice_contents(helpers, fake_push)!r}")
        if flag_path().exists():
            fail("matching 0301 suppress should clear flag")
        if window._pending_plan_push is not None:
            fail(f"matching 0301 suppress should clear pending push: {window._pending_plan_push!r}")
        if window._scheduled_0301_plan_ids:
            fail(f"matching 0301 suppress should clear scheduled 0301 ids: {window._scheduled_0301_plan_ids!r}")
        if window._post_0301_delivery is not None:
            fail(f"matching 0301 suppress should clear post-0301 delivery: {window._post_0301_delivery!r}")
        if window._post_0301_timer.isActive():
            fail("matching 0301 suppress should stop post-0301 timer")
        if "delivery suppressed before 0301" not in log_text(window):
            fail(f"matching 0301 suppress log changed: {log_text(window)!r}")


def check_stale_flag_allows_normal_delivery(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[9999])
        window = make_window(gui, helpers)
        window._pending_plan_push = pending_payload(5101)
        window._scheduled_0301_plan_ids = [5101]

        window._start_push_sequence()

        if msg_ids(helpers, fake_push) != ["0301", "0305"]:
            fail(f"stale flag should allow normal 0301/0305 before mode-ready: {msg_ids(helpers, fake_push)!r}")
        if flag_path().exists():
            fail("stale flag should still be read and cleared")
        if "flagPlans" not in log_text(window):
            fail(f"stale flag plan mismatch log changed: {log_text(window)!r}")
        if "0001" in msg_ids(helpers, fake_push):
            fail(f"stale flag should not push 0001: {fake_push.calls!r}")
        pending = window._post_0301_delivery
        if not isinstance(pending, dict) or not pending.get("completion_ready"):
            fail(f"stale flag should leave normal post-0301 pending delivery: {pending!r}")


def check_post_0301_matching_suppresses_flush(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        window = make_window(gui, helpers)
        window._pending_plan_push = pending_payload(5101)
        window._scheduled_0301_plan_ids = [5101]

        window._start_push_sequence()
        if msg_ids(helpers, fake_push) != ["0301", "0305"]:
            fail(f"post-0301 setup should send 0301/0305 first: {msg_ids(helpers, fake_push)!r}")
        if not isinstance(window._post_0301_delivery, dict):
            fail("post-0301 setup should leave pending delivery before mode-ready")

        write_flag(plan_ids=[5101])
        if not window._mark_post_0301_ready(trigger="smoke_mode_ready"):
            fail("post-0301 matching suppress should report handled flush")

        if msg_ids(helpers, fake_push) != ["0301", "0305", "0001"]:
            fail(f"post-0301 suppress should only add 0001: {msg_ids(helpers, fake_push)!r}")
        if flag_path().exists():
            fail("post-0301 suppress should clear flag")
        if window._post_0301_delivery is not None:
            fail(f"post-0301 suppress should clear pending delivery: {window._post_0301_delivery!r}")
        if "delivery suppressed before post-0301" not in log_text(window):
            fail(f"post-0301 suppress log changed: {log_text(window)!r}")


def check_0305_status1_does_not_consume(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers)

        if not window._push_0305(status=1, reason="planning started"):
            fail("0305 status=1 should still be sent")
        if msg_ids(helpers, fake_push) != ["0305"]:
            fail(f"0305 status=1 should not be suppress-converted: {msg_ids(helpers, fake_push)!r}")
        if not flag_path().exists():
            fail("0305 status=1 should not consume suppress flag")


def check_0305_status2_matching_suppresses(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers)

        if window._push_0305(status=2, reason="planning complete"):
            fail("0305 status=2 should return False when suppress flag matches")
        if msg_ids(helpers, fake_push) != ["0001"]:
            fail(f"0305 status=2 suppress should only push 0001: {msg_ids(helpers, fake_push)!r}")
        if flag_path().exists():
            fail("0305 status=2 suppress should clear flag")
        if "status=2 suppressed" not in log_text(window):
            fail(f"0305 status=2 suppress log changed: {log_text(window)!r}")


def check_0305_completion_wrapper_drops_pending(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers)
        window._queue_post_0301_delivery(
            plan_ids=[5101],
            option_names=["option1"],
            plan_meta={},
            is_execution_mode=True,
            force_direct=False,
            suppress_0702_fallback=False,
        )
        window._post_0301_timer.active = True

        if window._push_post_0301_completion(reason="planning complete"):
            fail("post-0301 completion wrapper should return False when 0305 is suppressed")
        if msg_ids(helpers, fake_push) != ["0001"]:
            fail(f"0305 completion wrapper suppress should only push 0001: {msg_ids(helpers, fake_push)!r}")
        if window._post_0301_delivery is not None:
            fail(f"0305 completion wrapper should drop pending delivery: {window._post_0301_delivery!r}")
        if window._post_0301_timer.isActive():
            fail("0305 completion wrapper should stop timer when suppressed")
        if any(name == "post_0301_completion_failed" for name, _extra in window._timing_events):
            fail(
                "0305 suppress currently clears pending inside the consume path before the wrapper "
                f"can record post_0301_completion_failed: {window._timing_events!r}"
            )


def check_0901_matching_suppresses_options(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers)
        window._pending_plan_push = {"kept": True}

        window._push_0901_options([5101], ["option1"], {})

        if msg_ids(helpers, fake_push) != ["0001"]:
            fail(f"0901 suppress should only push 0001: {msg_ids(helpers, fake_push)!r}")
        if flag_path().exists():
            fail("0901 suppress should clear flag")
        if window._pending_plan_push != {"kept": True}:
            fail(f"0901 suppress should not clear pending push directly: {window._pending_plan_push!r}")
        if "option suppressed" not in log_text(window):
            fail(f"0901 suppress log changed: {log_text(window)!r}")


def check_non_attack_0901_direct_reader_current_behavior(gui: Any, helpers: Any) -> None:
    fake_push = helpers.FakePushCenter()
    with temp_db(), helpers.patched_environment(gui, fake_push):
        write_flag(plan_ids=[5101])
        window = make_window(gui, helpers, active_ctx=non_attack_context())

        window._push_0901_options([5101], ["option1"], {})

        if msg_ids(helpers, fake_push) != ["0001"]:
            fail(
                "non-attack _push_0901_options currently reads matching flag and sends only 0001: "
                f"{msg_ids(helpers, fake_push)!r}"
            )
        if flag_path().exists():
            fail("non-attack _push_0901_options should clear flag in current behavior")
        if "option suppressed" not in log_text(window):
            fail(f"non-attack _push_0901_options suppress log changed: {log_text(window)!r}")


def main() -> int:
    helpers = load_delivery_smoke()
    try:
        gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
        check_non_attack_context_does_not_consume(gui, helpers)
        check_matching_0301_suppresses_full_delivery(gui, helpers)
        check_stale_flag_allows_normal_delivery(gui, helpers)
        check_post_0301_matching_suppresses_flush(gui, helpers)
        check_0305_status1_does_not_consume(gui, helpers)
        check_0305_status2_matching_suppresses(gui, helpers)
        check_0305_completion_wrapper_drops_pending(gui, helpers)
        check_0901_matching_suppresses_options(gui, helpers)
        check_non_attack_0901_direct_reader_current_behavior(gui, helpers)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        cleanup_process_console_state()
        helpers.cleanup_process_console_state()

    print("attack delivery suppress flag smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
