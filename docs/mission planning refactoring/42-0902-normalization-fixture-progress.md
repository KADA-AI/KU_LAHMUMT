# 0902 Normalization Fixture Progress

## Scope

This checkpoint freezes the baseline 0902 parse and selection-normalization behavior without changing runtime code. It is intentionally narrower than the later 0902 TODOs for ID priority, malformed fallback, dict-only mission IDs, and trigger/delay exact-match cases.

## Added

- `smoke_0902_normalization_fixture.py`

## Contract Captured

- `parse_replan_payload()` extracts an embedded JSON object from raw bytes with prefix/suffix noise.
- `parse_replan_payload()` extracts an embedded JSON object from text input.
- Mapping input is returned as an equal shallow copy, not the original object.
- Empty, missing, non-JSON, or non-mapping payloads return `None`.
- `extract_replan_request_selection(sample_0902)` does not mutate the input fixture.
- The sample fixture normalizes to:
  - `plan_ids == [700000001]`
  - `option_names == ["baseline-option"]`
  - `mission_ids == [1, 2]`
  - `detail is payload["replanDetail"]`
  - `detail_trigger_type == "communicationLossRTB"`
- Empty selection defaults remain `ReplanRequestSelection()`.

## Why This Is Safe

No runtime code changed. The smoke imports only the extracted replan request helper module and uses the existing `fixtures/payloads/sample_0902.json`.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_normalization_fixture.py"
python "docs\mission planning refactoring\smoke_0902_normalization_fixture.py"
```

Result:

```text
0902 normalization fixture smoke ok
```

## Next

Next incomplete TODO: `0902 ID extraction priority fixture 작성: optionList/pendingOptionList -> missionPlanIDList -> replanDetail.missionPlanID`.
