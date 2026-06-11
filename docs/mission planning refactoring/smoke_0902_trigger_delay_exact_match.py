from __future__ import annotations

import ast
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def delay_for(*, trigger: Any = None, trigger_type: Any = None, detail: Any = None) -> tuple[str | None, int]:
    from modules.mission_planning.app.message_handlers.replan_requests import replan_delay_policy

    if detail is None:
        detail = {"trigger": trigger, "triggerType": trigger_type}
    policy = replan_delay_policy({"replanDetail": detail})
    return policy.runtime_setting_key, policy.default_delay_ms


def policy_for(payload: dict[str, Any]) -> tuple[str | None, int]:
    from modules.mission_planning.app.message_handlers.replan_requests import replan_delay_policy

    policy = replan_delay_policy(payload)
    return policy.runtime_setting_key, policy.default_delay_ms


def check_delay_policy_exact_matches() -> None:
    cases = (
        ("non-mapping detail", {"detail": "bad"}, (None, 100)),
        (
            "collab refresh wins before attack trigger",
            {"trigger": "0402", "trigger_type": "collabReexecuteInputRefresh"},
            ("replan_collab_reexecute_schedule_delay_ms", 30),
        ),
        ("case-sensitive collab mismatch", {"trigger": "0402", "trigger_type": "collabreexecuteinputrefresh"}, (None, 0)),
        ("attack 0402", {"trigger": "0402", "trigger_type": ""}, (None, 0)),
        ("attack closed destroyed type", {"trigger": "0401", "trigger_type": "attackClosedDestroyed"}, (None, 0)),
        ("attack closed destroyed wins with non-0401 trigger", {"trigger": "0201", "trigger_type": "attackClosedDestroyed"}, (None, 0)),
        ("communication loss rtb", {"trigger": "0401", "trigger_type": "communicationLossRTB"}, (None, 55_000)),
        ("abnormal health rtb", {"trigger": "0401", "trigger_type": "abnormalHealthRTB"}, (None, 55_000)),
        ("unexpected rtb", {"trigger": "0401", "trigger_type": "unexpectedRTB"}, (None, 55_000)),
        ("unknown 0401 type", {"trigger": "0401", "trigger_type": "unknown"}, (None, 100)),
        ("non-0401 non-attack trigger", {"trigger": "0201", "trigger_type": "communicationLossRTB"}, (None, 100)),
        ("stripped trigger and type", {"trigger": " 0401 ", "trigger_type": " communicationLossRTB "}, (None, 55_000)),
        ("case-sensitive type mismatch", {"trigger": "0401", "trigger_type": "communicationlossrtb"}, (None, 100)),
        ("case-sensitive attack type mismatch", {"trigger": "0401", "trigger_type": "attackcloseddestroyed"}, (None, 100)),
        ("case-sensitive trigger mismatch", {"trigger": "attackClosedDestroyed", "trigger_type": ""}, (None, 100)),
    )
    if policy_for({}) != (None, 100):
        fail(f"0902 missing replanDetail policy changed: {policy_for({})!r}")
    for label, payload, expected in cases:
        if label == "non-mapping detail":
            actual = delay_for(detail=payload["detail"])
        else:
            actual = delay_for(trigger=payload["trigger"], trigger_type=payload["trigger_type"])
        if actual != expected:
            fail(f"0902 delay policy changed for {label}: {actual!r} != {expected!r}")


def check_gui_delay_wrapper_contract() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    source = gui_path.read_text(encoding="utf-8-sig", errors="ignore")
    tree = ast.parse(source)

    target = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_replan_delay_ms_for_payload":
            target = node
            break
    if target is None:
        fail("mission_planning_gui missing _replan_delay_ms_for_payload")

    text = ast.get_source_segment(source, target) or ""
    expected_fragments = (
        "policy = replan_delay_policy(payload)",
        "if policy.runtime_setting_key:",
        "self._runtime_replan_delay_ms(policy.runtime_setting_key, policy.default_delay_ms)",
        "return int(policy.default_delay_ms)",
    )
    for fragment in expected_fragments:
        if fragment not in text:
            fail(f"mission GUI delay wrapper changed, missing: {fragment}")


def main() -> int:
    try:
        check_delay_policy_exact_matches()
        check_gui_delay_wrapper_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 trigger/delay exact-match smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
