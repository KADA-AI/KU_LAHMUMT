# Current Remaining Hybrid Fallback PathID Progress

## Scope

This checkpoint freezes the current-remaining hybrid failure fallback and pathID remap behavior before any folder moves or wrapper cleanup.

## Added

- `smoke_current_remaining_hybrid_fallback_pathids.py`
- `fixtures/current_remaining_hybrid/failure_fallback_pathid_mapping.json`

## Contract Captured

- `materialize_current_remaining_hybrid_result` keeps the pathID-to-aircraft mapping from replacement missions and infers missing 0303 `aircraftID` values from that map.
- `materialize_current_remaining_hybrid_result` returns `None` when the prepared hybrid has no replacement missions or no generated flight paths.
- `current_replan.prepare_current_remaining_hybrid_replacements` returns `None` for invalid current input IDs, no available UAVs, no live UAV coordinates, and empty next-collab replacements.
- `current_replan.prepare_current_remaining_hybrid_replacements` also returns `None` when the next-collab helper itself returns `None`.
- GUI call sites only call `filter_generic_flightpath_missions_for_hybrid` after a hybrid result exists; failed hybrid builds keep the generic output.
- Successful hybrid application removes only the replaced current generic FlightPath mission.
- Preserved generic pathIDs that collide with hybrid-generated pathIDs cause `temporaryPathIdRemap` instead of leaving overlapping 0303/0304 pathIDs.
- GUI source markers still show pathID mapping is delayed until after current-remaining hybrid merge in both sequential and parallel-store paths.
- `validate_current_remaining_hybrid_paths` continues to report generic/hybrid overlaps explicitly.
- `mission_planning_gui.py` still records `flightpath_missing_ids` when expected pathIDs are not present in generated FlightPath artifacts.

## Boundary

This smoke does not run the planner, create GUI windows, touch DB payload directories, or generate artifacts. It imports the remaining-hybrid modules, uses an in-repo JSON fixture, and monkeypatches only in-process helper functions for failure branches.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_current_remaining_hybrid_fallback_pathids.py"
python "docs\mission planning refactoring\smoke_current_remaining_hybrid_fallback_pathids.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
current remaining hybrid fallback/pathID smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: wrapper template consolidation.
