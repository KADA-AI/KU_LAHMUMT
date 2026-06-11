# Replan Dispatcher Semantics Progress

## Scope

This checkpoint freezes current replan dispatcher priority and handled/fallback semantics.

## Added

- `smoke_replan_dispatcher_semantics.py`

## Contract Captured

- `SPECIALIZED_DISPATCH_ORDER` remains:
  - `post_attack_rejoin`
  - `next_collab`
  - `imaging_schedule`
  - `path_deviation`
  - `prior`
- Post-attack rejoin is an exact `trigger == "0402"` and `triggerType == "attackClosedDestroyed"` match.
- Prior post-rejoin is an exact `trigger == "0401"` and `triggerType == "priorClosedResume"` match.
- Post-attack rejoin suppresses the normal attack route.
- Non-rejoin `0402`, attack reason keywords, and attack option-name keywords still route to the attack pipeline.
- String `option_names` is ignored as a scalar, not iterated as a sequence of names.
- GUI specialized dispatch order remains attack, post-attack rejoin, next-collab, imaging-schedule, path-deviation, prior.
- Prior post-rejoin is attempted inside prior handling before `run_prior_mission_pipeline(...)`.
- False-route helper calls return `(False, None)`.
- When post-attack/prior-post route predicates match and the specialized pipeline returns no result, the helper returns handled `True` with the current skipped summary shape.

## Why This Is Safe

No runtime code changed. The smoke calls pure dispatcher predicates and uses fake pipeline functions only inside the test process for handled/no-result helper semantics.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_replan_dispatcher_semantics.py"
python "docs\mission planning refactoring\smoke_replan_dispatcher_semantics.py"
```

Result:

```text
replan dispatcher semantics smoke ok
```

## Next

Next incomplete TODO: `monitoring queue priority/source-plan rebound/suppress semantics fixture 작성`.
