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


class SignalSink:
    def __init__(self) -> None:
        self.items: list[Any] = []

    def emit(self, *args: Any) -> None:
        self.items.append(args)


class PipelineLogger:
    def __init__(self) -> None:
        self.items: list[Any] = []

    def log_event(self, *args: Any, **kwargs: Any) -> None:
        self.items.append((args, kwargs))


def fail(message: str) -> None:
    raise SmokeFailure(message)


def check_dispatcher_predicates() -> None:
    from modules.mission_planning.replanning import dispatcher

    if dispatcher.SPECIALIZED_DISPATCH_ORDER != (
        dispatcher.POST_ATTACK_REJOIN_ROUTE,
        "next_collab",
        "imaging_schedule",
        "path_deviation",
        "prior",
    ):
        fail(f"dispatcher specialized order changed: {dispatcher.SPECIALIZED_DISPATCH_ORDER!r}")

    post_detail = {"trigger": "0402", "triggerType": "attackClosedDestroyed"}
    prior_detail = {"trigger": "0401", "triggerType": "priorClosedResume"}
    attack_detail = {"trigger": "0402", "triggerType": "attackOption"}

    if not dispatcher.is_post_attack_rejoin_detail(post_detail):
        fail("post-attack rejoin detail predicate changed")
    if dispatcher.is_post_attack_rejoin_detail({"trigger": "0402", "triggerType": "attackcloseddestroyed"}):
        fail("post-attack rejoin detail predicate is no longer exact-case")
    if not dispatcher.is_prior_post_rejoin_detail(prior_detail):
        fail("prior post-rejoin detail predicate changed")
    if dispatcher.is_prior_post_rejoin_detail({"trigger": "0401", "triggerType": "priorclosedresume"}):
        fail("prior post-rejoin detail predicate is no longer exact-case")

    if not dispatcher.should_use_post_attack_rejoin_pipeline({"replan_detail": post_detail}):
        fail("post-attack route predicate changed")
    if dispatcher.should_use_attack_pipeline({"replan_detail": post_detail}):
        fail("post-attack rejoin no longer suppresses attack pipeline route")
    if not dispatcher.should_use_attack_pipeline({"replan_detail": attack_detail}):
        fail("0402 non-rejoin detail no longer routes to attack pipeline")
    if not dispatcher.should_use_attack_pipeline({"reason": "\uacf5\uaca9 \ud2b9\ud654 request"}):
        fail("attack reason keyword route changed")
    if not dispatcher.should_use_attack_pipeline({"option_names": ["normal", "\uacf5\uaca9\ucd94\ucc9c"]}):
        fail("attack option-name keyword route changed")
    if dispatcher.should_use_attack_pipeline({"option_names": "\uacf5\uaca9\ucd94\ucc9c"}):
        fail("string option_names no longer ignored as a sequence")
    if not dispatcher.should_use_prior_post_rejoin_pipeline({"replan_detail": prior_detail}):
        fail("prior post-rejoin route predicate changed")


def check_gui_dispatch_source_order() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    source = gui_path.read_text(encoding="utf-8-sig", errors="ignore")
    fragments = {
        "attack": "use_attack_pipeline = self._should_use_attack_pipeline(ctx)",
        "post_attack": "post_attack_handled, post_attack_summary = self._try_run_post_attack_rejoin_pipeline(",
        "next_collab": "next_collab_handled, next_collab_summary = self._try_run_next_collab_replan_pipeline(",
        "imaging": "imaging_schedule_handled, imaging_schedule_summary = self._try_run_imaging_schedule_replan_pipeline(",
        "path": "path_deviation_handled, path_deviation_summary = self._try_run_path_deviation_replan_pipeline(",
        "prior": "prior_summary = self._try_run_prior_mission_pipeline(ctx, reason, session_id=session_id)",
    }
    positions: dict[str, int] = {}
    for key, fragment in fragments.items():
        try:
            positions[key] = source.index(fragment)
        except ValueError:
            fail(f"GUI dispatch source missing fragment: {fragment}")
    if not (
        positions["attack"]
        < positions["post_attack"]
        < positions["next_collab"]
        < positions["imaging"]
        < positions["path"]
        < positions["prior"]
    ):
        fail(f"GUI specialized dispatch order changed: {positions!r}")

    tree = ast.parse(source)
    prior_func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_try_run_prior_mission_pipeline":
            prior_func = node
            break
    if prior_func is None:
        fail("GUI missing _try_run_prior_mission_pipeline")
    prior_text = ast.get_source_segment(source, prior_func) or ""
    if prior_text.index("self._try_run_prior_post_rejoin_pipeline(") > prior_text.index("result = run_prior_mission_pipeline("):
        fail("prior post-rejoin is no longer handled before prior mission pipeline")
    if "if prior_post_rejoin_handled:\n            return prior_post_rejoin_summary" not in prior_text:
        fail("prior post-rejoin handled return semantics changed")


def make_window(gui: Any):
    window = gui.MainWindow.__new__(gui.MainWindow)
    window.log_sig = SignalSink()
    window._pipeline_logger = PipelineLogger()
    window._push_replan_noop_completion = lambda *_args, **_kwargs: None
    return window


def check_gui_helper_handled_semantics() -> None:
    import importlib

    gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
    window = make_window(gui)

    if window._try_run_post_attack_rejoin_pipeline({}, "reason") != (False, None):
        fail("post-attack helper false-route semantics changed")
    if window._try_run_prior_post_rejoin_pipeline({}, "reason") != (False, None):
        fail("prior post-rejoin helper false-route semantics changed")

    original_post = gui.run_post_attack_rejoin_pipeline
    original_prior = gui.run_prior_post_rejoin_pipeline
    try:
        gui.run_post_attack_rejoin_pipeline = lambda *_args, **_kwargs: None
        gui.run_prior_post_rejoin_pipeline = lambda *_args, **_kwargs: None

        post_ctx = {"replan_detail": {"trigger": "0402", "triggerType": "attackClosedDestroyed"}}
        post_handled, post_summary = window._try_run_post_attack_rejoin_pipeline(post_ctx, "reason")
        if not post_handled or not post_summary or post_summary.get("reason") != "pipeline_returned_none":
            fail(f"post-attack handled/no-result semantics changed: {(post_handled, post_summary)!r}")

        prior_ctx = {
            "replan_level": 4,
            "replan_detail": {"trigger": "0401", "triggerType": "priorClosedResume"},
        }
        prior_handled, prior_summary = window._try_run_prior_post_rejoin_pipeline(prior_ctx, "reason")
        if (
            not prior_handled
            or not prior_summary
            or prior_summary.get("reason") != "pipeline_returned_none"
            or prior_summary.get("mode") != "priorPostRejoin"
        ):
            fail(f"prior post-rejoin handled/no-result semantics changed: {(prior_handled, prior_summary)!r}")
    finally:
        gui.run_post_attack_rejoin_pipeline = original_post
        gui.run_prior_post_rejoin_pipeline = original_prior


def main() -> int:
    try:
        check_dispatcher_predicates()
        check_gui_dispatch_source_order()
        check_gui_helper_handled_semantics()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("replan dispatcher semantics smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
