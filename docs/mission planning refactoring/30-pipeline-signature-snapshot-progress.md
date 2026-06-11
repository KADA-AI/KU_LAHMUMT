# 30. Pipeline Signature Snapshot Progress

## Scope

이번 수정은 Phase 0의 주요 `run_*` pipeline signature snapshot 작성 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_pipeline_signatures.py`

## Contract

스크립트는 다음 6개 public run entrypoint의 현재 `inspect.signature()` 문자열을 고정한다.

- `run_attack_plan_pipeline`
- `run_prior_mission_pipeline`
- `run_next_collab_replan_pipeline`
- `run_path_deviation_replan_pipeline`
- `run_imaging_schedule_replan_pipeline`
- `run_post_attack_rejoin_pipeline`

이번 항목은 call contract snapshot만 다룬다. Pipeline 실행 결과 shape, fixture replay, wrapper import identity는 각각 별도 smoke/TODO에서 다룬다.

## Snapshot Notes

- attack pipeline은 `(ctx, log_callback=None)` 형태를 유지한다.
- prior/next-collab/path-deviation/imaging-schedule pipeline은 `(ctx, detail, reason, *, log)` 형태를 유지한다.
- post-attack rejoin pipeline은 `(ctx, detail, reason, *, log=None)` 형태를 유지한다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_pipeline_signatures.py"`
- `python "docs\mission planning refactoring\smoke_pipeline_signatures.py"`

## Next TODO

다음 미완료 TODO는 `d0301/d0302/d0303/d0304` public builder signature snapshot 작성이다.
