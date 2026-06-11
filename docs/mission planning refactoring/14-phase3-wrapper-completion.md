# Phase 3 Wrapper Completion

Date: 2026-06-04, Asia/Seoul

## Completed

- The final Phase 3 wrapper checkbox was completed as:
  - `이동 완료된 기존 pipelines/*.py는 wrapper로 유지`
- `smoke_import_contract.py` now verifies wrapper shape for moved old pipeline paths:
  - each old path references the canonical trigger module
  - each old path has no top-level function/class definitions
  - wrapper identity still matches canonical objects

## Moved Old Pipeline Paths Covered

- `pipelines/attack_plan_pipeline.py`
- `pipelines/prior_mission_pipeline_impl.py`
- `pipelines/next_collab_replan_pipeline.py`
- `pipelines/next_collab_replan_pipeline_impl.py`
- `pipelines/current_remaining_hybrid.py`
- `pipelines/current_remaining_hybrid_replan.py`
- `pipelines/general_remaining_hybrid_replan.py`
- `pipelines/reexecute_first_mission_hybrid.py`
- `pipelines/recon_specialized_pipeline.py`
- `pipelines/imaging_schedule_replan_pipeline_impl.py`
- `pipelines/path_deviation_replan_pipeline_impl.py`
- `pipelines/post_attack_rejoin_pipeline.py`

## Still Active Support Files

These are not trigger implementation wrappers yet and must stay active:

- `pipelines/mission_path_trim.py`
- `pipelines/mission_planning_attack_helpers.py`
- `pipelines/next_collab_path_builder.py`

## Progress Snapshot

- Overall roadmap after this checkbox: 24 / 99 complete, 75 remaining, 24.2% complete.
- Phase 3: 11 / 11 complete, 0 remaining, 100.0% complete.
