# Recon Specialized Reload Policy Progress

## Scope

`recon_specialized_pipeline.py`의 hot-reload 정책을 현행 동작 기준으로 고정했다. 이번 항목은 runtime code를 수정하지 않고, recon-specialized helper가 watch/reload 대상이지만 GUI globals rebinding 대상은 아닌 현재 계약을 문서화하고 smoke로 검증한다.

## Added

- `smoke_recon_specialized_reload_policy.py`

## Current Policy

- Canonical implementation:
  - `modules.mission_planning.replanning.triggers.recon_specialized.pipeline`
- Compatibility wrapper:
  - `modules.mission_planning.pipelines.recon_specialized_pipeline`
- Watch path:
  - `modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py`
- Reload module:
  - `modules.mission_planning.replanning.triggers.recon_specialized.pipeline`
- Globals rebinding:
  - Not included in `PIPELINE_RELOAD_BINDINGS`
  - Not explicitly rebound by `refresh_live_planning_helpers(namespace)`

This means a source change to the canonical recon helper is part of planner source signature detection and causes the planner runtime refresh path to reload the module, but the direct GUI globals `build_recon_specialized_runtime_payload` and `is_recon_specialized_option` are not replaced during hot-reload. That is the current behavior and is now treated as the baseline contract.

Changing the old compatibility wrapper itself does not trigger planner hot reload because the wrapper file is not in the watch list. It may still affect legacy/external consumers that import `modules.mission_planning.pipelines.recon_specialized_pipeline`, so wrapper identity/export behavior remains covered by smoke.

## Contract Captured

- The recon canonical file remains in `PLANNER_RUNTIME_WATCH_RELATIVE_PATHS`.
- The recon canonical module remains in `PLANNER_RUNTIME_RELOAD_ORDER`.
- The recon canonical module remains absent from `PIPELINE_RELOAD_BINDINGS`.
- `refresh_live_planning_helpers(namespace)` reloads the recon module but does not mutate stale recon helper names in the supplied namespace.
- If the supplied namespace lacks recon helper names, refresh does not create those keys.
- The compatibility wrapper exports the canonical functions by object identity.
- The `replanning.triggers.recon_specialized` package exports the canonical functions by object identity.
- `mission_planning_gui.py` still imports `build_recon_specialized_runtime_payload` and `is_recon_specialized_option` as direct globals, and its refresh bridge does not add a recon-specific rebinding block.

## Why This Is Safe

No runtime code changed. The smoke uses a fake `reload_planning_module` only inside the test process and restores it in `finally`. It does not execute mission planning scenarios or invoke GUI startup.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_recon_specialized_reload_policy.py"
python "docs\mission planning refactoring\smoke_recon_specialized_reload_policy.py"
python "docs\mission planning refactoring\smoke_planner_hot_reload_snapshot.py"
python "docs\mission planning refactoring\smoke_planner_rebinding_fixture.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
py_compile: pass
recon specialized watch/reload policy smoke ok
planner hot-reload watch/reload snapshot smoke ok
planner hot-reload globals rebinding fixture smoke ok
mission planning refactor import-contract smoke ok
git diff --check: pass
```

Read-only sub-agent review also confirmed that the new smoke covers the current recon watch/reload-without-GUI-rebind baseline.

## Next

Next incomplete TODO: `bootstrap import-order contract 작성: KU_ROLE=mission, console/file logging 설치 전 side-effect 금지`.
