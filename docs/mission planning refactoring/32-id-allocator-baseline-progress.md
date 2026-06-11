# 32. ID Allocator Baseline Progress

## Scope

이번 수정은 Phase 0의 ID allocator counter 파일과 reserve API baseline 기록 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_id_allocator_baseline.py`

## Counter File Baseline

ID allocator의 canonical 구현은 다음 모듈이다.

- `modules.mission_planning.engine.mission_generation.id_allocation.allocator`

보존해야 하는 counter/store 계약은 다음과 같다.

- legacy fallback store는 `modules/mission_planning/MissionPlanner/data_def/id_tracker.json`이다.
- runtime active store는 `DSS_Internal/id_tracker.json` 아래로 resolve된다.
- canonical engine allocator 폴더 옆에는 `id_tracker.json`을 만들지 않는다.
- file lock은 store 파일명 기준 `id_tracker.json.lock`을 사용한다.
- path/waypoint usage side artifact는 active DB의 `DSS_Internal/path_usage.json`, `DSS_Internal/waypoint_usage.json`이다.

## Reserve API Baseline

현재 `BASE` 값은 다음과 같다.

- `missionPlanID`: `700000001`
- `individualMissionPackage`: `800000001`
- `individualMission`: `900000001`
- `pathID`: aircraft 1~6 각각 `100000001`, `200000001`, `300000001`, `400000001`, `500000001`, `600000001`
- `waypoint`: `50`

격리 temp store에서 확인한 public reserve API의 현재 반환 baseline은 다음과 같다.

- `reserve_mission_plan_ids(2)` -> `[700000001, 700000002]`
- `reserve_imp_ids(2)` -> `[800000001, 800000002]`
- `reserve_individual_mission_ids(1)` -> `[900000001]`
- `reserve_path_ids(1, 2)` -> `[100000001, 100000002]`
- `reserve_path_id_blocks({2: 2, "3": 1, 4: 0})` -> `{2: [200000001, 200000002], 3: [300000001]}`
- `reserve_replan_id_bundle()` -> empty `pathID`, `individualMissionPackage`, `individualMission`
- `reserve_replan_id_bundle(path_count_by_aircraft={4: 2}, imp_count=1, individual_mission_count=2)` -> path 4 block `[400000001, 400000002]`, IMP `[800000003]`, individual `[900000002, 900000003]`
- `reserve_waypoint_block(3)` -> `50`
- `reserve_waypoint_blocks([2, 0, "1"])` -> `[(53, 54), (55, 55)]`

현행 예외 baseline:

- `reserve_path_ids(99, 1)`은 `KeyError(99)`를 낸다.
- `reserve_waypoint_block(0)`은 `ValueError`를 낸다.

## Wrapper Baseline

다음 wrapper는 canonical allocator와 같은 function object를 export해야 한다.

- `modules.mission_planning.MissionPlanner.data_def.id_allocator`
- `data_def.id_allocator`

`modules.mission_planning.MissionPlanner.data_def.id_allocator.__file__`은 기존 wrapper 파일 경로를 유지한다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_id_allocator_baseline.py"`
- `python "docs\mission planning refactoring\smoke_id_allocator_baseline.py"`

## Next TODO

다음 미완료 TODO는 sample 0201/0203/0902 payload fixture 확보이다.
