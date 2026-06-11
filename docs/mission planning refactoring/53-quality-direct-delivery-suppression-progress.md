# Quality Direct Delivery Suppression Progress

## Scope

This checkpoint freezes the current quality-speed and direct-delivery suppression matrix for `0901` and `0702`.

## Added

- `smoke_quality_direct_delivery_suppression.py`

## Contract Captured

- Quality-speed reason matching uses the current exact Korean keyword text.
- Quality-speed trigger matching is exact `qualityMonitorSep`.
- Quality-speed detection works from direct `plan_meta.triggerType` and nested `plan_meta.replanDetail.triggerType`.
- Scheduling delivery with a quality-speed reason marks `quality_speed`, forces direct delivery, suppresses `0702`, and sends `0301 -> 0305 -> 0903` only.
- The current scheduling timing metric still records one sorted/default option name before `_start_push_sequence()` applies the quality direct-delivery suppression.
- Scheduling delivery with quality-speed plan meta has the same forced direct/no-`0702` behavior.
- `_push_0901_options(...)` blocks option creation when quality-speed is present in plan meta, active-context reason, or active-context trigger detail.
- Pending option-mode delivery is still converted to direct `0903` at flush time when its `plan_meta` is quality-speed.
- Imaging direct delivery sends `0702` unless its suppress flag is set.
- Imaging quality direct delivery, next-collab direct delivery, and explicit suppress direct delivery suppress `0702`.
- Path-deviation direct and prior direct keep the current `0903 -> 0702` fallback path.
- Multi-plan prior direct delivery keeps sorted `0301`, `0903`, and `0702` plan order and schedules fallback delays as `0, 250, 200, 450`.

## Boundary

This smoke does not cover attack delivery suppress flags or post-delivery waypoint/snapshot carry-forward. Those remain separate TODO items.

## Why This Is Safe

No runtime code changed. The smoke reuses the fake `push_message` and immediate timer harness from the delivery order smoke, patches the suppress-flag reader to return `None`, and restores all patches after execution.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_quality_direct_delivery_suppression.py"
python "docs\mission planning refactoring\smoke_quality_direct_delivery_suppression.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
quality direct delivery suppression smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `attack delivery suppress flag smoke`.
