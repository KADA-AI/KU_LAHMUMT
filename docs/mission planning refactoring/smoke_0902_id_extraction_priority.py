from __future__ import annotations

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


def selection_for(payload: dict[str, Any]):
    from modules.mission_planning.app.message_handlers.replan_requests import (
        extract_replan_request_selection,
    )

    return extract_replan_request_selection(payload)


def check_option_list_wins() -> None:
    selection = selection_for(
        {
            "optionList": [
                {"missionPlanID": "700000101", "optionName": "option-a"},
                {"missionPlanID": 700000102, "optionName": "option-b"},
            ],
            "pendingOptionList": [
                {"missionPlanID": 700000201, "optionName": "pending-a"},
            ],
            "missionPlanIDList": [{"missionPlanID": 700000301}],
            "replanDetail": {"missionPlanID": 700000401},
        }
    )
    if selection.plan_ids != [700000101, 700000102]:
        fail(f"optionList priority changed: {selection.plan_ids!r}")
    if selection.option_names != ["option-a", "option-b"]:
        fail(f"optionList optionName extraction changed: {selection.option_names!r}")


def check_pending_option_list_wins_when_option_list_empty_or_missing() -> None:
    for label, option_value in (("missing", None), ("empty", [])):
        payload: dict[str, Any] = {
            "pendingOptionList": [
                {"missionPlanID": "700000211", "optionName": "pending-a"},
                {"missionPlanID": 700000212, "optionName": "pending-b"},
            ],
            "missionPlanIDList": [{"missionPlanID": 700000311}],
            "replanDetail": {"missionPlanID": 700000411},
        }
        if option_value is not None:
            payload["optionList"] = option_value
        selection = selection_for(payload)
        if selection.plan_ids != [700000211, 700000212]:
            fail(f"pendingOptionList priority changed for {label} optionList: {selection.plan_ids!r}")
        if selection.option_names != ["pending-a", "pending-b"]:
            fail(f"pendingOptionList optionName extraction changed for {label}: {selection.option_names!r}")


def check_mission_plan_id_list_fallback() -> None:
    selection = selection_for(
        {
            "optionList": [],
            "pendingOptionList": [],
            "missionPlanIDList": [
                {"missionPlanID": "700000321"},
                700000322,
                {"missionPlanID": "bad"},
            ],
            "replanDetail": {"missionPlanID": 700000421},
        }
    )
    if selection.plan_ids != [700000321, 700000322]:
        fail(f"missionPlanIDList fallback changed: {selection.plan_ids!r}")
    if selection.option_names != []:
        fail(f"missionPlanIDList fallback unexpectedly produced option names: {selection.option_names!r}")


def check_replan_detail_fallback() -> None:
    selection = selection_for(
        {
            "optionList": [],
            "pendingOptionList": [],
            "missionPlanIDList": [],
            "replanDetail": {"missionPlanID": "700000431"},
        }
    )
    if selection.plan_ids != [700000431]:
        fail(f"replanDetail missionPlanID fallback changed: {selection.plan_ids!r}")
    if selection.option_names != []:
        fail(f"replanDetail fallback unexpectedly produced option names: {selection.option_names!r}")

    invalid = selection_for({"replanDetail": {"missionPlanID": 0}})
    if invalid.plan_ids != []:
        fail(f"non-positive replanDetail missionPlanID no longer ignored: {invalid.plan_ids!r}")


def main() -> int:
    try:
        check_option_list_wins()
        check_pending_option_list_wins_when_option_list_empty_or_missing()
        check_mission_plan_id_list_fallback()
        check_replan_detail_fallback()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 ID extraction priority smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
