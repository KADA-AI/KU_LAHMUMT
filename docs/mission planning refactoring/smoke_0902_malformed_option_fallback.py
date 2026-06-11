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


def check_malformed_option_list_does_not_fallback_to_pending_list() -> None:
    selection = selection_for(
        {
            "optionList": [
                {"missionPlanID": "bad", "optionName": "bad-option"},
                {"optionName": "missing-plan-id"},
            ],
            "pendingOptionList": [
                {"missionPlanID": 700000201, "optionName": "valid-pending"},
            ],
            "missionPlanIDList": [{"missionPlanID": 700000301}],
            "replanDetail": {"missionPlanID": 700000401},
        }
    )
    if selection.plan_ids != [700000301]:
        fail(
            "malformed truthy optionList no longer skips pendingOptionList before missionPlanIDList fallback: "
            f"{selection.plan_ids!r}"
        )
    if selection.option_names != []:
        fail(f"malformed optionList unexpectedly produced option names: {selection.option_names!r}")


def check_malformed_option_list_without_mission_plan_list_falls_to_detail_not_pending() -> None:
    selection = selection_for(
        {
            "optionList": [
                {"missionPlanID": None, "optionName": "bad-option"},
            ],
            "pendingOptionList": [
                {"missionPlanID": 700000211, "optionName": "valid-pending"},
            ],
            "missionPlanIDList": [],
            "replanDetail": {"missionPlanID": 700000411},
        }
    )
    if selection.plan_ids != [700000411]:
        fail(
            "malformed truthy optionList no longer skips pendingOptionList before replanDetail fallback: "
            f"{selection.plan_ids!r}"
        )
    if selection.option_names != []:
        fail(f"malformed optionList/detail fallback unexpectedly produced option names: {selection.option_names!r}")


def check_truthy_non_list_option_list_also_skips_pending_list() -> None:
    selection = selection_for(
        {
            "optionList": "bad-non-list",
            "pendingOptionList": [
                {"missionPlanID": 700000221, "optionName": "valid-pending"},
            ],
            "missionPlanIDList": [{"missionPlanID": 700000321}],
        }
    )
    if selection.plan_ids != [700000321]:
        fail(f"truthy non-list optionList no longer skips pendingOptionList: {selection.plan_ids!r}")
    if selection.option_names != []:
        fail(f"truthy non-list optionList unexpectedly produced option names: {selection.option_names!r}")


def main() -> int:
    try:
        check_malformed_option_list_does_not_fallback_to_pending_list()
        check_malformed_option_list_without_mission_plan_list_falls_to_detail_not_pending()
        check_truthy_non_list_option_list_also_skips_pending_list()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 malformed optionList fallback smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
