from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "payloads" / "sample_0902.json"


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def load_sample_0902() -> dict[str, Any]:
    payload = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        fail("sample_0902 fixture is not a JSON object")
    return payload


def check_parse_baseline() -> None:
    from modules.mission_planning.app.message_handlers.replan_requests import parse_replan_payload

    sample = load_sample_0902()
    raw = ("prefix\n" + json.dumps(sample, ensure_ascii=False) + "\nsuffix").encode("utf-8")
    parsed = parse_replan_payload(raw)
    if parsed != sample:
        fail("0902 raw bytes parse no longer extracts the embedded JSON object")

    parsed_from_text = parse_replan_payload("noise " + json.dumps(sample, ensure_ascii=False))
    if parsed_from_text != sample:
        fail("0902 text parse no longer extracts the embedded JSON object")

    parsed_from_mapping = parse_replan_payload(sample)
    if parsed_from_mapping != sample:
        fail("0902 mapping parse no longer preserves payload content")
    if parsed_from_mapping is sample:
        fail("0902 mapping parse no longer returns a copy")

    for invalid in (None, b"", "no-json-here", ["not", "mapping"]):
        if parse_replan_payload(invalid) is not None:
            fail(f"0902 invalid payload no longer returns None: {invalid!r}")


def check_sample_selection_normalization() -> None:
    from modules.mission_planning.app.message_handlers.replan_requests import (
        ReplanRequestSelection,
        extract_replan_request_selection,
    )

    sample = load_sample_0902()
    before = copy.deepcopy(sample)
    selection = extract_replan_request_selection(sample)

    if not isinstance(selection, ReplanRequestSelection):
        fail(f"0902 selection type changed: {type(selection)!r}")
    if sample != before:
        fail("0902 selection extraction mutated the input payload")
    if selection.plan_ids != [700000001]:
        fail(f"0902 sample normalized plan IDs changed: {selection.plan_ids!r}")
    if selection.option_names != ["baseline-option"]:
        fail(f"0902 sample normalized option names changed: {selection.option_names!r}")
    if selection.mission_ids != [1, 2]:
        fail(f"0902 sample normalized mission IDs changed: {selection.mission_ids!r}")
    if selection.detail is not sample.get("replanDetail"):
        fail("0902 selection no longer preserves the original replanDetail object")
    if selection.detail_trigger_type != "communicationLossRTB":
        fail(f"0902 detail trigger type changed: {selection.detail_trigger_type!r}")


def check_empty_selection_defaults() -> None:
    from modules.mission_planning.app.message_handlers.replan_requests import (
        ReplanRequestSelection,
        extract_replan_request_selection,
    )

    selection = extract_replan_request_selection({})
    if selection != ReplanRequestSelection():
        fail(f"0902 empty selection defaults changed: {selection!r}")


def main() -> int:
    try:
        check_parse_baseline()
        check_sample_selection_normalization()
        check_empty_selection_defaults()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("0902 normalization fixture smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
