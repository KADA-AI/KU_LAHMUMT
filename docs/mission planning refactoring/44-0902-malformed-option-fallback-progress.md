# 0902 Malformed Option Fallback Progress

## Scope

This checkpoint freezes the current behavior where a truthy malformed `optionList` does not fall back to a valid `pendingOptionList`.

## Added

- `smoke_0902_malformed_option_fallback.py`

## Contract Captured

- A truthy `optionList` is selected before `pendingOptionList`.
- If that selected `optionList` is a list but yields no valid `missionPlanID`, `pendingOptionList` is not consulted.
- In that case, extraction falls through to `missionPlanIDList` if it can produce IDs.
- If `missionPlanIDList` also yields no IDs, extraction falls through to positive `replanDetail.missionPlanID`.
- A truthy non-list `optionList` also skips `pendingOptionList` and falls through to `missionPlanIDList`.
- Malformed selected option entries do not produce `option_names`.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_malformed_option_fallback.py"
python "docs\mission planning refactoring\smoke_0902_malformed_option_fallback.py"
```

Result:

```text
0902 malformed optionList fallback smoke ok
```

## Next

Next incomplete TODO: `0902 inputMissionIDList dict-only 추출 fixture 작성`.
