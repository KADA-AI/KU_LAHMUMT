# 0902 Trigger Delay Exact Match Progress

## Scope

This checkpoint freezes the current exact-match delay policy for 0902 `replanDetail.trigger` and `replanDetail.triggerType`.

## Added

- `smoke_0902_trigger_delay_exact_match.py`

## Contract Captured

- Missing or non-mapping `replanDetail` returns default delay `100` ms.
- `triggerType == "collabReexecuteInputRefresh"` wins before attack handling and returns runtime setting key `replan_collab_reexecute_schedule_delay_ms` with default `30` ms.
- Wrong-case `collabReexecuteInputRefresh` does not match.
- `trigger == "0402"` returns `0` ms.
- `triggerType == "attackClosedDestroyed"` returns `0` ms.
- `attackClosedDestroyed` wins even when `trigger != "0401"`.
- Non-`0401` non-attack triggers return `100` ms.
- `trigger == "0401"` with `triggerType` in `communicationLossRTB`, `abnormalHealthRTB`, `unexpectedRTB` returns `55000` ms.
- Unknown `0401` trigger types return `100` ms.
- `trigger` and `triggerType` are stripped before matching.
- Matching is case-sensitive.
- `mission_planning_gui.py` delegates to `replan_delay_policy(payload)` and uses `_runtime_replan_delay_ms(...)` only when `policy.runtime_setting_key` is present.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_trigger_delay_exact_match.py"
python "docs\mission planning refactoring\smoke_0902_trigger_delay_exact_match.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
```

Result:

```text
py_compile: pass
0902 trigger/delay exact-match smoke ok
mission planning refactor import-contract smoke ok
```

Read-only sub-agent review found missing branch-order and wrong-case checks; the smoke was updated to include them. Runtime override clamping and immediate/deferred delay consumption are intentionally left to the next `trigger별 0902 delay/deferred queue fixture` TODO.

## Next

Next incomplete TODO: `trigger별 0902 delay/deferred queue fixture 작성`.
