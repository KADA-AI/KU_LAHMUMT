# 0902 ID Extraction Priority Progress

## Scope

This checkpoint freezes the current `extract_replan_request_selection()` mission-plan ID priority for 0902 payloads. It does not cover the malformed `optionList` behavior; that remains the next dedicated TODO.

## Added

- `smoke_0902_id_extraction_priority.py`

## Contract Captured

- A valid non-empty `optionList` wins over `pendingOptionList`, `missionPlanIDList`, and `replanDetail.missionPlanID`.
- If `optionList` is missing or an empty list, a valid non-empty `pendingOptionList` wins over `missionPlanIDList` and `replanDetail.missionPlanID`.
- If option lists are empty, `missionPlanIDList` is used.
- `missionPlanIDList` accepts both dict entries with `missionPlanID` and scalar entries.
- Invalid `missionPlanIDList` entries are skipped.
- If no earlier source yields IDs, positive `replanDetail.missionPlanID` is used.
- Non-positive `replanDetail.missionPlanID` is ignored.
- `option_names` are collected only from the selected option-list source.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_id_extraction_priority.py"
python "docs\mission planning refactoring\smoke_0902_id_extraction_priority.py"
```

Result:

```text
0902 ID extraction priority smoke ok
```

## Next

Next incomplete TODO: `0902 malformed optionList가 valid pendingOptionList로 fallback되지 않는 현행 동작 fixture 작성`.
