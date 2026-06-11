# External Import Contract Inventory Progress

## Scope

This checkpoint freezes the non-mission-planning code that imports or launches mission planning internals before further folder moves.

## Added

- `smoke_external_import_contract.py`

## Contract Captured

- `app/ui/main_window.py`:
  - imports `MissionPlanner.runtime_settings` for FOV DB path configuration.
  - still searches `modules/mission_planning/mission_planning_gui.py` and the legacy `app/modules/...` fallback launcher path.
- `run.py`:
  - still resets mission-planning ID tracker files through `MissionPlanner/data_def`.
  - still imports `id_allocator` and `id_allocator_0202` for cold-start reset.
  - still reaches into allocator private state and file locations: `__file__`, `BASE`, `_state`, `_volatile_counters`, `VOLATILE_KEYS`, `_BASE_STATE`, and `_STATE`.
  - still maps the mission role to `mission_planning_gui.py`.
- `modules/common`:
  - `agent_status_snapshot.py` still updates mission-planning attack/prior tracking state with optional fallbacks.
  - `next_collab_replan_store.py` still re-exports the active mission-planning next-collab detail/event store.
  - `button_wiring.py` still maps the assignment card to `mission_planning_gui.py`.
  - `ops_checklist.py` still maps `mission_planning_gui.py`, `mission_planning`, `mission`, and `mmr` to assignment/MMR semantics.
- `modules/monitoring`:
  - monitoring GUI and logic still depend on mission-planning runtime settings.
  - target detection and monitoring GUI keep optional fallbacks for attack assignment/tracking state imports.
  - prior mission replan still uses attack lineage and prior handoff state.
  - next-collab monitoring still imports mission-planning runtime/store constants.
  - mission visualization still imports `DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS`.
  - initial replan still allocates mission plan IDs through the legacy `MissionPlanner/data_def/id_allocator` path.

## Boundary

This does not refactor the external imports and does not execute monitoring, dashboard, or run.py. It is a source/AST inventory that defines what wrappers or public contracts must remain valid while moving mission-planning internals.

The broader `modules/sim` import of `MissionPlanner.data_def.mission_helpers.terrain_elev` is recorded as adjacent risk, but it is not part of this monitoring/common/app TODO and should be handled only if a later deletion or move touches simulator-facing mission-planning paths.

Project-root bare-import shims `config.py`, `AnS/`, and `data_def/` are intentionally absent. Bare `AnS`, `data_def`, and `config` imports remain covered by the existing import-contract smoke through the internal `MissionPlanner` path bootstrap rather than this external inventory.

## Why This Is Safe

No runtime code changed. The smoke parses monitored files with `utf-8-sig` to handle BOM-marked modules, compares the current external `modules.mission_planning...` import inventory, and checks launcher strings by source markers.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_external_import_contract.py"
python "docs\mission planning refactoring\smoke_external_import_contract.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
monitoring/common/app external import contract smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `logic_test, tool GUI, portable bundle manual workflow owner decision`.
