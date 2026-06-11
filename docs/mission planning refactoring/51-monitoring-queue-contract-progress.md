# Monitoring Queue Contract Progress

## Scope

This checkpoint freezes the current monitoring replan queue ordering, source-plan rebound, and target-detection option suppression contracts before larger mission-planning refactors.

## Added

- `smoke_monitoring_queue_contract.py`

## Contract Captured

- Queue plan-id extraction prefers `pendingOptionList`, then `optionList`, then `missionPlanIDList`, then root `missionPlanID`.
- Target-detection enqueue ordering keeps attack-close payloads first, then applies configured target-type priority, while non-target sources keep input order.
- `target_dispatch_delay_ms` delays the first target-detection dispatch until `ready_at_ms`.
- When queued items share the same ready time, non-target requests are promoted before normal target-detection requests.
- Queued normal target-detection requests collapse into the existing queued target item and carry the latest target/plan payload.
- Attack-close `0402` payloads preempt an active normal target-detection request once it has reached `planning_finished`, `options_requested`, or `options_sent`.
- While a normal target-detection request is active, a newer normal target-detection request sets `suppress_options` on the active request and writes `DSS_Internal/suppress_option_request.json`.
- The suppress flag records active `target_id`, `target_key`, `queue_id`, `plan_ids`, `created_ms`, and `active=True`.
- Completing a suppressed active request clears the suppress flag and promotes the queued request.
- Once `0701` has moved the active item to `options_sent`/`options_delivered`, a later target-detection request no longer sets a suppress flag.
- GUI source-plan rebound keeps attack options on `_last_mission_plan_id`, and post-attack rejoin options fall back through `sourcePlanID` then `currentMissionPlanID`.
- Follow-up attack detail preparation writes both `sourceMissionPlanID` and `currentMissionPlanID` from the rebounded source plan.
- GUI suppress matching rejects stale flags by disjoint plan ids, mismatched target id, and mismatched target key.
- Monitoring GUI dispatch preparation rebounds `0402` and `0401` payload `sourceMissionPlanID/currentMissionPlanID` to the current dispatch plan, falling back to the visualization plan id when needed.
- Target-detection and RTB monitoring payload builders still populate both `sourceMissionPlanID` and `currentMissionPlanID` from `current_mission_plan_id`.

## Why This Is Safe

No runtime code changed. The smoke uses a temporary monkeypatched DB subpath for the suppress flag and creates `MainWindow` with `__new__` plus stub logging only; it does not start Qt, send messages, or touch `current_scenario.json`.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_monitoring_queue_contract.py"
python "docs\mission planning refactoring\smoke_monitoring_queue_contract.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
monitoring queue contract smoke ok
mission planning refactor import-contract smoke ok
```

## Pause Note

Per request, work pauses after this checkpoint. Next incomplete TODO: `delivery order matrix and fake push_message smoke`.
