# Planner Hot-Reload Rebinding Fixture Progress

## Scope

`planner_runtime.refresh_live_planning_helpers(namespace)`의 현재 hot-reload rebinding 계약을 스모크로 고정했다. 이번 항목은 watch list나 reload order의 목록 자체가 아니라, reload 이후 `mission_planning_gui.py`의 `globals()` namespace에 걸린 오래된 함수/클래스 참조가 어떻게 교체되는지를 다룬다.

## Added

- `smoke_planner_rebinding_fixture.py`

## Contract Captured

- `importlib.invalidate_caches()`가 rebinding 전에 1회 호출된다.
- `PLANNER_RUNTIME_RELOAD_ORDER` 순서대로 `reload_planning_module(module_name)`가 호출된다.
- `remaining_hybrid.current`의 명시적 globals rebinding 대상:
  - `CurrentRemainingHybridRequest`
  - `build_current_remaining_hybrid`
  - `merge_current_remaining_hybrid`
  - `validate_current_remaining_hybrid_request`
  - `validate_current_remaining_hybrid_paths`
  - `filter_generic_flightpath_missions_for_hybrid`
- `runtime.aircraft_parallel_0303`의 명시적 globals rebinding 대상:
  - `build_0303_flight_plans_aircraft_parallel`
- `PIPELINE_RELOAD_BINDINGS`에 등록된 모든 pipeline/warmup 함수가 namespace의 stale 값을 새 모듈 값으로 교체한다.
- reload 결과가 `None`인 모듈은 namespace를 건드리지 않는다.
- reload된 모듈이 특정 attr을 제공하지 않으면 기존 namespace 값을 유지한다.
- reload된 모듈이 특정 attr을 제공하지 않고 namespace key도 없으면 현재 구현처럼 해당 attr key를 `None`으로 기록한다.
- `reload_planning_module()`은 import 실패 시 `None`, reload 실패 시 import된 module object, reload 성공 시 reloaded module object를 반환한다.

## Why This Is Safe

스모크는 실제 mission planning pipeline을 실행하거나 heavy module reload를 유발하지 않는다. `planner_runtime.reload_planning_module`과 `importlib.invalidate_caches`를 fixture 내부에서만 임시 대체하고, `finally`에서 원복한다. 따라서 현재 기능 경로에는 변화가 없고, hot-reload rebinding 계약만 회귀 감지 대상으로 추가된다.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_planner_rebinding_fixture.py" "docs\mission planning refactoring\smoke_import_contract.py"
python "docs\mission planning refactoring\smoke_planner_rebinding_fixture.py"
python "docs\mission planning refactoring\smoke_planner_hot_reload_snapshot.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
py_compile: pass
planner hot-reload globals rebinding fixture smoke ok
planner hot-reload watch/reload snapshot smoke ok
git diff --check: pass
```

`python "docs\mission planning refactoring\smoke_import_contract.py"` was also retried with a 60s timeout during this pause point, but it timed out without a failure message. The new rebinding fixture itself and the existing hot-reload snapshot both passed.

## Pause Note

사용자 요청에 따라 이번 수정은 여기까지 진행하고 중단한다. 다음 미완료 TODO는 `recon_specialized_pipeline.py watch-only/reload 정책 결정 및 smoke 작성`이다.
