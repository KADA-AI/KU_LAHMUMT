# 0902 Input Mission IDs Progress

## Scope

This checkpoint freezes current `inputMissionIDList` extraction behavior in `extract_replan_request_selection()`.

## Added

- `smoke_0902_input_mission_ids.py`

## Contract Captured

- `inputMissionIDList` must be a list to be considered.
- Each list item is treated as a dict-like object; only `item.get("inputMissionID")` is read.
- Dict entries with numeric strings are converted with `int(...)`.
- Scalar list entries are ignored.
- Dict entries with other keys such as `missionID` are ignored.
- Invalid `inputMissionID` values are skipped.
- Current implementation does not filter signed or zero values after integer conversion.
- Extraction does not mutate the input payload.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_input_mission_ids.py"
python "docs\mission planning refactoring\smoke_0902_input_mission_ids.py"
```

Result:

```text
0902 inputMissionIDList extraction smoke ok
```

## Next

Next incomplete TODO: `0902 trigger/delay exact-match fixture 작성`.
