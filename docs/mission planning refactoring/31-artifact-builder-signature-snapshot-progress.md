# 31. Artifact Builder Signature Snapshot Progress

## Scope

이번 수정은 Phase 0의 `d0301/d0302/d0303/d0304` public builder signature snapshot 작성 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_artifact_builder_signatures.py`

## Contract

스크립트는 canonical artifact builder package의 public builder/helper 함수 9개에 대해 현재 `inspect.signature()` 문자열을 고정한다.

- `d0301.build_mission_plan`
- `d0302.build_mission_packages`
- `d0303.build_flight_plans`
- `d0303.set_flyover_options`
- `d0303.reset_dense_linesearch_metrics`
- `d0303.get_dense_linesearch_metrics`
- `d0304.build_lah_flight_plans_fixed`
- `d0304.build_lah_flight_plans_from_mrpk`
- `d0304.apply_uav_eta_follow_speed_plan`

각 함수는 canonical module과 기존 `modules.mission_planning.MissionPlanner.data_def.d030N`, bare `data_def.d030N` wrapper에서 같은 function object로 유지되어야 한다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_artifact_builder_signatures.py"`
- `python "docs\mission planning refactoring\smoke_artifact_builder_signatures.py"`

## Next TODO

다음 미완료 TODO는 ID allocator counter 파일과 reserve API baseline 기록이다.
