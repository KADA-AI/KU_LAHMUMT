# 29. Pipeline Import Smoke Progress

## Scope

이번 수정은 Phase 0의 주요 pipeline import smoke 정의 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_pipeline_imports.py`

## Contract

스크립트는 pipeline을 실행하지 않고 import/export 계약만 확인한다. 다음 TODO가 signature snapshot이므로 이번 항목에서는 `inspect.signature`나 fixture 실행은 하지 않는다.

확인하는 계약은 다음과 같다.

- attack, prior, next-collab, imaging-schedule, path-deviation, post-attack trigger pipeline의 run/warm entrypoint가 import 가능하고 callable이다.
- 주요 result class가 class object로 유지된다.
- current/general/reexecute-first remaining hybrid와 recon-specialized helper entrypoint가 import 가능하고 callable이다.
- mission path trim, attack helper, next-collab path builder 같은 pipeline support module의 주요 helper가 import 가능하다.
- 현행 public wrapper, legacy wrapper, `pipelines/` compatibility wrapper가 canonical object와 같은 함수/class object를 export한다.
- wrapper `__all__`이 있는 경우 현재 public export 범위 안의 symbol을 빠뜨리지 않는다.

## Current Public Wrapper Boundary

`modules.mission_planning.pipelines.next_collab_replan_pipeline`은 현재 `NextCollabPipelineResult`, `run_next_collab_replan_pipeline`, `warm_next_collab_replan_pipeline`만 public export한다. `prepare_next_collab_input_replacements`는 broad implementation wrapper인 `modules.mission_planning.pipelines.next_collab_replan_pipeline_impl`에서 보존한다. 기능 변화를 피하기 위해 이 현행 차이를 smoke 계약에 반영했다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_pipeline_imports.py"`
- `python "docs\mission planning refactoring\smoke_pipeline_imports.py"`

## Next TODO

다음 미완료 TODO는 `run_attack_plan_pipeline`, `run_prior_mission_pipeline`, `run_next_collab_replan_pipeline`, `run_path_deviation_replan_pipeline`, `run_imaging_schedule_replan_pipeline`, `run_post_attack_rejoin_pipeline` signature snapshot 작성이다.
