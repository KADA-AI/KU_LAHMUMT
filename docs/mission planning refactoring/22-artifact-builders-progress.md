# 22. Artifact Builders Progress

## Scope

이번 수정은 `MissionPlanner/data_def/d0301.py`부터 `d0304.py`까지의 0301/0302/0303/0304 산출물 생성 구현을 mission generation engine 영역으로 이동한 작업이다.

새 canonical 위치:

- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py`

기존 import 계약은 제거하지 않았다. 아래 내부 경로는 compatibility wrapper로 유지하고, bare `data_def.d030N` import는 project-root `data_def/` shim이 아니라 `MissionPlanner` path bootstrap으로 해결한다.

- `modules/mission_planning/MissionPlanner/data_def/d0301.py` to `d0304.py`
- bare `data_def.d0301` to `data_def.d0304` import contract through `MissionPlanner` path bootstrap

## Compatibility Contract

보존한 계약:

- `modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d030N`
- `modules.mission_planning.MissionPlanner.data_def.d030N`
- `data_def.d030N`

위 세 import 경로는 같은 module object를 반환해야 한다. 특히 `d0303`과 `d0304`는 GUI와 `planning_enhanced`가 `importlib.reload()` 및 module global assignment를 사용하므로, `from ... import *` 래퍼를 사용하지 않고 `sys.modules[__name__] = canonical_module` 방식으로 유지했다.

보존한 주요 public symbols:

- `d0301`: `build_mission_plan`
- `d0302`: `build_mission_packages`
- `d0303`: `build_flight_plans`, `set_flyover_options`, `reset_dense_linesearch_metrics`, `get_dense_linesearch_metrics`, `_WPAllocator`, `SweepConfig`
- `d0304`: `build_lah_flight_plans_fixed`, `build_lah_flight_plans_from_mrpk`, `apply_uav_eta_follow_speed_plan`

## Risk Handling

- `d0301._COUNTER_FILE`은 기존 `MissionPlanner/data_def/_id_counters.json` 위치를 유지한다.
- `d0303._FOV_DB_PATH`는 기존 의미대로 repo root의 `resource/db/fov_db.csv`를 가리키게 유지한다.
- `d0304`의 RL/DEM path 계산은 이동 후에도 repo root `resource`와 기존 `MissionPlanner/portable_mission_bundle` 기준으로 유지한다.
- `mission_control/planner_runtime.py` watch list에 새 canonical artifact builder 파일을 추가해서, wrapper가 아니라 실제 구현 파일 수정도 runtime cache invalidation에 반영되게 했다.

## Verification Added

`smoke_import_contract.py`에 다음 검증을 추가했다.

- 새 artifact builder 파일 존재 확인
- old/root wrapper shape 확인
- canonical/old/bare import module identity 확인
- public symbol identity 확인
- engine-first, old-first, bare-first import order 확인
- `d0303`, `d0304` reload 후 identity 유지 확인
- `d0301` counter path와 `d0303` FOV DB path 확인
- planner runtime watch list에 canonical artifact builder 경로 포함 확인

## Current Status

이번 항목은 완료 처리했다. 남은 Phase 5 항목은 `AnS/mission_pipeline.py` 이동 전 path bootstrap과 dynamic reload 목록 수정이다.
