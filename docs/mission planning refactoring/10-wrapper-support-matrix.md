# Wrapper Support Matrix

Date: 2026-06-04

This matrix freezes the import paths that must keep working while trigger pipelines move from `pipelines/` into `replanning/triggers/`.

## Moved Trigger Pipelines

| Trigger | Canonical implementation | Supported compatibility paths | Unsupported paths | Export policy |
| --- | --- | --- | --- | --- |
| Attack | `modules.mission_planning.replanning.triggers.attack.pipeline` | `modules.mission_planning.attack_plan_pipeline`, `modules.mission_planning.legacy.wrappers.attack_plan_pipeline`, `modules.mission_planning.pipelines.attack_plan_pipeline` | none currently identified | Broad non-dunder re-export for root/legacy/pipelines wrappers. |
| Prior mission | `modules.mission_planning.replanning.triggers.prior.pipeline` | `modules.mission_planning.prior_mission_pipeline`, `modules.mission_planning.legacy.wrappers.prior_mission_pipeline`, `modules.mission_planning.prior_mission_pipeline_impl`, `modules.mission_planning.legacy.wrappers.prior_mission_pipeline_impl`, `modules.mission_planning.pipelines.prior_mission_pipeline_impl` | none currently identified | Root/legacy entrypoint wrappers expose run/warm only. Impl wrappers broadly re-export non-dunder symbols because other pipelines still use private helpers. |
| Next collaborative mission | `modules.mission_planning.replanning.triggers.next_collab.pipeline` | `modules.mission_planning.next_collab_replan_pipeline`, `modules.mission_planning.legacy.wrappers.next_collab_replan_pipeline`, `modules.mission_planning.pipelines.next_collab_replan_pipeline`, `modules.mission_planning.pipelines.next_collab_replan_pipeline_impl` | none currently identified | Root/legacy expose run/warm only. `pipelines.next_collab_replan_pipeline` exposes result class plus run/warm. Impl wrapper broadly re-exports non-dunder symbols. Support modules remain in `pipelines/` and `runtime/`. |
| Remaining hybrid | `modules.mission_planning.replanning.triggers.remaining_hybrid.current`, `modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan`, `modules.mission_planning.replanning.triggers.remaining_hybrid.general` | `modules.mission_planning.pipelines.current_remaining_hybrid`, `modules.mission_planning.pipelines.current_remaining_hybrid_replan`, `modules.mission_planning.pipelines.general_remaining_hybrid_replan` | none currently identified | Old `pipelines` files broadly re-export non-dunder symbols. GUI and planner runtime use the canonical current/general modules. |
| Reexecute-first helper | `modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first` | `modules.mission_planning.pipelines.reexecute_first_mission_hybrid` | none currently identified | Kept under `remaining_hybrid` because it feeds execute=2 first-mission hybrid replacement. Old path broadly re-exports non-dunder symbols. |
| Recon specialized | `modules.mission_planning.replanning.triggers.recon_specialized.pipeline` | `modules.mission_planning.pipelines.recon_specialized_pipeline` | none currently identified | Recon option/runtime payload helper. Old path broadly re-exports non-dunder symbols. |
| Imaging schedule | `modules.mission_planning.replanning.triggers.imaging_schedule.pipeline` | `modules.mission_planning.imaging_schedule_replan_pipeline`, `modules.mission_planning.legacy.wrappers.imaging_schedule_replan_pipeline`, `modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl` | none currently identified | Root/legacy expose run/warm only. `pipelines` wrapper also exposes result class and trigger constants. |
| Path deviation | `modules.mission_planning.replanning.triggers.path_deviation.pipeline` | `modules.mission_planning.path_deviation_replan_pipeline`, `modules.mission_planning.legacy.wrappers.path_deviation_replan_pipeline`, `modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl` | none currently identified | Root/legacy expose run/warm only. `pipelines` wrapper also exposes result class. |
| Post-attack rejoin | `modules.mission_planning.replanning.triggers.post_attack.pipeline` | `modules.mission_planning.pipelines.post_attack_rejoin_pipeline` | `modules.mission_planning.post_attack_rejoin_pipeline` | Broad non-dunder re-export for the `pipelines` wrapper. There was no historical root wrapper. |

## Not Yet Physically Moved

| Trigger | Current implementation | New trigger entrypoint | Current compatibility paths | Notes |
| --- | --- | --- | --- | --- |
| Support helpers | `modules.mission_planning.pipelines.mission_path_trim`, `modules.mission_planning.pipelines.next_collab_path_builder` | none yet | Existing `pipelines` imports only | Still active support modules. Do not delete. |

## Guardrails

- Moved implementation paths must not be reintroduced as internal imports:
  - `modules.mission_planning.pipelines.attack_plan_pipeline`
  - `modules.mission_planning.pipelines.prior_mission_pipeline_impl`
  - `modules.mission_planning.pipelines.next_collab_replan_pipeline_impl`
  - `modules.mission_planning.pipelines.current_remaining_hybrid`
  - `modules.mission_planning.pipelines.current_remaining_hybrid_replan`
  - `modules.mission_planning.pipelines.general_remaining_hybrid_replan`
  - `modules.mission_planning.pipelines.reexecute_first_mission_hybrid`
  - `modules.mission_planning.pipelines.recon_specialized_pipeline`
  - `modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl`
  - `modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl`
  - `modules.mission_planning.pipelines.post_attack_rejoin_pipeline`
- Existing wrappers may continue importing canonical trigger modules.
- Shared helper dependencies under `pipelines/` are still allowed until they are moved with their own wrappers.
- `smoke_import_contract.py` verifies that moved old `pipelines/*.py` files reference canonical modules and no longer define functions/classes.
- `modules/mission_planning/app` must stay a namespace package without `__init__.py` unless the top-level repository `app` package is renamed or otherwise protected.
- Before staging a PR, run `python "docs/mission planning refactoring/smoke_import_contract.py" --require-git-tracked` so newly extracted files cannot be omitted.
