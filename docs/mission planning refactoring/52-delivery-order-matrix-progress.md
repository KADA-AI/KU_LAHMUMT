# Delivery Order Matrix Progress

## Scope

This checkpoint freezes the current 0301/0305/0901/0903/0702 delivery ordering with a fake `push_message` before larger mission-planning delivery refactors.

## Added

- `smoke_delivery_order_matrix.py`

## Contract Captured

- Delivery plan entries are deduplicated, sorted by `missionPlanID`, and keep their matching option names.
- Post-0301 delay helpers keep option-mode and force-direct mode names and timeout clamping behavior.
- Pending delivery requests merge into one pending payload without adding duplicate plan IDs.
- Option execution mode sends `0301` for each sorted plan, then `0305 status=2`, and waits for mode-ready before sending `0901`.
- Apply mode sends `0301` for each sorted plan, then `0305 status=2`, and waits for mode-ready before sending one `0903` per sorted plan.
- Apply-mode `0903` sends are scheduled at `idx * 200ms` and do not send `0702`.
- Basic force-direct delivery sends `0301 -> 0305 -> 0903 -> 0702` for one plan when `suppress_0702_fallback=False`.
- Basic force-direct `0903` and `0702` timers keep the current `0ms` and `250ms` spacing.
- Force-direct delivery with `suppress_0702_fallback=True` sends `0301 -> 0305 -> 0903` and records `0702_suppressed`.
- Missing `missionPlanID` blocks all push calls and currently leaves the pending payload intact.

## Boundary

This smoke intentionally avoids the next TODO's deeper quality-speed/direct suppression matrix and attack suppress-flag behavior. It only captures the base direct order and one explicit suppress flag branch needed by the delivery matrix.

## Why This Is Safe

No runtime code changed. The smoke creates `MainWindow` with `__new__`, installs a temporary fake top-level `push_center` module, stubs timers to execute immediately, and restores all patches after execution.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_delivery_order_matrix.py"
python "docs\mission planning refactoring\smoke_delivery_order_matrix.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
delivery order matrix smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `quality-speed/direct delivery matrix and 0901/0702 suppression smoke`.
