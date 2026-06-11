# Recon And Reexecute Progress

Date: 2026-06-04, Asia/Seoul

## Completed

- Moved reexecute-first helper:
  - Canonical: `modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first`
  - Compatibility wrapper: `modules.mission_planning.pipelines.reexecute_first_mission_hybrid`
  - Reason: this helper feeds execute=2 first-mission replacement inside remaining hybrid flow.
- Moved recon-specialized helper:
  - Canonical: `modules.mission_planning.replanning.triggers.recon_specialized.pipeline`
  - Compatibility wrapper: `modules.mission_planning.pipelines.recon_specialized_pipeline`
  - Reason: this helper owns recon option detection and recon runtime payload shaping.
- Updated imports:
  - `mission_planning_gui.py`
  - `MissionPlanner/planning_enhanced/pipeline.py`
  - `replanning/triggers/remaining_hybrid/current.py`
  - `mission_control/planner_runtime.py`
- Updated `smoke_import_contract.py`:
  - verifies wrapper identity for reexecute/recon helpers
  - forbids direct imports of the old `pipelines` implementation paths
  - verifies planner runtime watch/reload coverage
- Checked the Phase 3 TODO for recon/reexecute helper movement.

## Verification Passed

- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `python -m compileall` for:
  - remaining_hybrid package
  - recon_specialized package
  - compatibility wrappers
  - planner runtime
  - mission planning GUI
  - planning enhanced pipeline
- Wrapper identity smoke:
  - `prepare_reexecute_first_mission_replacements`
  - `reexecute_first_mission_generic_skip_policy`
  - `validate_reexecute_first_mission_inputs`
  - `build_recon_specialized_runtime_payload`
  - `is_recon_specialized_option`
  - `summarize_recon_area_review_guard`
- Stale import search found no direct imports of:
  - `modules.mission_planning.pipelines.reexecute_first_mission_hybrid`
  - `modules.mission_planning.pipelines.recon_specialized_pipeline`

## TODO Progress Snapshot

- Overall roadmap: 23 / 99 complete, 76 remaining, 23.2% complete.
- Phase 3: 10 / 11 complete, 1 remaining, 90.9% complete.
- Phase 3 remaining checkbox: `기존 pipelines/*.py는 wrapper로 유지`.
