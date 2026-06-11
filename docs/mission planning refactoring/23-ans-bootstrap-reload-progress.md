# 23. AnS Bootstrap Reload Progress

## Scope

이번 수정은 `AnS/mission_pipeline.py`를 실제로 이동하기 전 필요한 path bootstrap과 dynamic reload 계약을 고정한 작업이다. `mission_pipeline.py` 구현 이동은 수행하지 않았다.

## Changes

- `mission_control/planner_runtime.py`에 mission planner import bootstrap path 목록을 명시했다.
- bootstrap 순서는 repo root, `modules`, `modules/mission_planning`, `modules/mission_planning/MissionPlanner`로 고정했다.
- planner runtime watch list는 canonical `modules/mission_planning/MissionPlanner/AnS/__init__.py`를 사용한다.
- planner runtime watch list에 `mission_pipeline.py`의 직접 support module인 `coord_transform.py`, `task_patterns_ver2.py`, `mission_effectiveness_ver2.py`, `env_patternselection.py`를 포함했다.
- dynamic reload order에 support modules를 `mission_pipeline.py`보다 먼저 추가했다.
- dynamic reload order에 `modules.mission_planning.MissionPlanner.AnS.mission_pipeline`과 `modules.mission_planning.MissionPlanner.AnS`를 추가했다.
- `AnS` package reload binding에 `run_divide_and_pattern`, `run_pulp_scheduling`, `build_mission_plan_0301`, `get_last_divide_and_pattern_metrics`를 고정했다.
- project-root `AnS` compatibility shim은 제거하고, path bootstrap 뒤 bare `AnS`가 canonical `modules.mission_planning.MissionPlanner.AnS` module object로 alias되도록 정리했다.
- canonical `MissionPlanner/AnS/__init__.py`와 `mission_pipeline.py`의 bare alias 등록은 stale alias를 덮어쓸 수 있도록 명시 assignment로 정리했다.

## Compatibility Contract

아래 import 경로는 같은 module object를 반환해야 한다.

- `AnS`
- `modules.mission_planning.MissionPlanner.AnS`

아래 submodule 경로도 같은 module object를 반환해야 한다.

- `AnS.mission_pipeline`
- `modules.mission_planning.MissionPlanner.AnS.mission_pipeline`

`AnS.mission_pipeline` reload 후에는 `AnS` package를 다시 reload해서 package-level exported function이 최신 submodule function object를 가리켜야 한다.

## Verification Added

`smoke_import_contract.py`에 다음 검증을 추가했다.

- `MISSION_PLANNER_IMPORT_RELATIVE_PATHS` 필수 bootstrap path 확인
- planner runtime watch list에 canonical `AnS` path 포함 확인
- planner runtime watch/reload order에 AnS support modules 포함 확인
- planner runtime reload order에 `AnS.mission_pipeline`, `AnS` package 포함 확인
- `AnS` package reload binding 확인
- root cwd와 `MissionPlanner` cwd에서 bare `AnS`/canonical `AnS` identity 확인
- `AnS`, canonical `AnS`, `AnS.mission_pipeline`, canonical `mission_pipeline` 중 어느 것을 먼저 import해도 module identity가 합쳐지는지 확인
- `mission_pipeline.DEM_PATH`와 `_ID_COUNTER_FILE`이 기존 `MissionPlanner/AnS` 위치를 유지하는지 확인
- 임의 cwd에서 `ensure_mission_planner_import_paths()` 호출 후 bootstrap path 순서와 `AnS` reload export freshness 확인

## Current Status

Phase 5의 마지막 TODO인 `AnS/mission_pipeline.py` 이동 전 path bootstrap과 dynamic reload 목록 수정은 완료 처리했다.
