# Attack Delivery Suppress Flag Progress

## Scope

This checkpoint freezes the current attack-delivery suppress-flag behavior around `0301`, `0305`, and `0901` delivery.

## Added

- `smoke_attack_delivery_suppress_flag.py`

## Contract Captured

- Non-attack active context does not read or clear `DSS_Internal/suppress_option_request.json`.
- Matching `0402` attack context before `0301` converts delivery to a single `0001` notice.
- Matching `0301` suppress clears `_pending_plan_push`, `_scheduled_0301_plan_ids`, `_post_0301_delivery`, stops the post-0301 timer, and clears the suppress flag.
- Stale suppress flags are read and cleared but do not block normal `0301 -> 0305` delivery.
- Stale plan mismatch logs still include `flagPlans`.
- Matching suppress flag at post-0301 flush blocks `0901/0903` and sends only `0001` after the already-sent `0301/0305`.
- `0305 status=1` does not consume the suppress flag.
- Matching `0305 status=2` suppresses `0305` and sends only `0001`.
- `_push_post_0301_completion(...)` sees suppressed `0305 status=2` as `False`; the consume path has already dropped pending post-0301 delivery and stopped the timer, so no `post_0301_completion_failed` timing event is recorded.
- Matching `0901` suppresses option delivery and sends only `0001`.
- The `0901` suppress path does not directly clear `_pending_plan_push`.
- Current `_push_0901_options(...)` reads the flag even for non-attack active context; if plan/target match, it suppresses `0901`, sends `0001`, and clears the flag.

## Boundary

This smoke does not cover post-delivery waypoint/snapshot carry-forward. It uses only fake `push_message`, immediate timers, and a temporary DB root for the suppress flag.

## Why This Is Safe

No runtime code changed. The smoke creates `MainWindow` with `__new__`, restores the real suppress consumer method on that fake instance, writes the suppress flag into an isolated temporary DB path, and restores all patches after execution.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_attack_delivery_suppress_flag.py"
python "docs\mission planning refactoring\smoke_attack_delivery_suppress_flag.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
attack delivery suppress flag smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `post-delivery waypoint mark/snapshot carry-forward smoke`.
