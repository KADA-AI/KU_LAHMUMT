from __future__ import annotations

import copy
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


def check_dict_only_input_mission_ids() -> None:
    payload: dict[str, Any] = {
        "optionList": [{"missionPlanID": 700000101, "optionName": "option-a"}],
        "inputMissionIDList": [
            {"inputMissionID": "1"},
            {"inputMissionID": 2},
            3,
            "4",
            {"missionID": 5},
            {"inputMissionID": "bad"},
            None,
        ],
    }
    before = copy.deepcopy(payload)
    selection = selection_for(payload)
    if payload != before:
        fail("0902 inputMissionIDList extraction mutated payload")
    if selection.mission_ids != [1, 2]:
        fail(f"0902 inputMissionIDList dict-only extraction changed: {selection.mission_ids!r}")


def check_non_list_input_mission_ids_are_ignored() -> None:
    for value in ({"inputMissionID": 1}, "1", 1, None):
        selection = selection_for({"inputMissionIDList": value})
        if selection.mission_ids != []:
            fail(f"0902 non-list inputMissionIDList no longer ignored for {value!r}: {selection.mission_ids!r}")


def check_signed_integer_conversion_is_preserved() -> None:
    selection = selection_for(
        {
            "inputMissionIDList": [
                {"inputMissionID": "-7"},
                {"inputMissionID": 0},
            ],
        }
    )
    if selection.mission_ids != [-7, 0]:
        fail(f"0902 inputMissionIDList int conversion/filtering changed: {selection.mission_ids!r}")


def main() -> int:
    try:
        check_dict_only_input_mission_ids()
        check_non_list_input_mission_ids_are_ignored()
        check_signed_integer_conversion_is_preserved()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 inputMissionIDList extraction smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
