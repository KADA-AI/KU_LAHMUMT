# Planner Hot-reload Watch/Reload Snapshot

## 목적

planner hot-reload의 watch list, reload order, binding map을 exact snapshot으로 고정했다. 이 항목은 "무엇을 감지하고 어떤 순서로 reload할지"를 다루며, 실제 `globals()` rebinding 동작 fixture는 다음 TODO에서 별도로 다룬다.

## 추가 파일

- `smoke_planner_hot_reload_snapshot.py`

## Import Path Snapshot

`planner_runtime.ensure_mission_planner_import_paths(project_root)`가 관리하는 path 순서:

1. `.`
2. `modules`
3. `modules/mission_planning`
4. `modules/mission_planning/MissionPlanner`

## Watch List Snapshot

현재 `PLANNER_RUNTIME_WATCH_RELATIVE_PATHS`:

1. `modules/mission_planning/MissionPlanner/AnS/__init__.py`
2. `modules/mission_planning/MissionPlanner/AnS/coord_transform.py`
3. `modules/mission_planning/MissionPlanner/AnS/task_patterns_ver2.py`
4. `modules/mission_planning/MissionPlanner/AnS/mission_effectiveness_ver2.py`
5. `modules/mission_planning/MissionPlanner/AnS/env_patternselection.py`
6. `modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py`
7. `modules/mission_planning/MissionPlanner/data_def/d0302.py`
8. `modules/mission_planning/MissionPlanner/data_def/d0303.py`
9. `modules/mission_planning/MissionPlanner/data_def/d0304.py`
10. `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py`
11. `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py`
12. `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py`
13. `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py`
14. `modules/mission_planning/replanning/triggers/remaining_hybrid/current.py`
15. `modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py`
16. `modules/mission_planning/replanning/triggers/remaining_hybrid/general.py`
17. `modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py`
18. `modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py`
19. `modules/mission_planning/pipelines/next_collab_path_builder.py`
20. `modules/mission_planning/replanning/triggers/next_collab/pipeline.py`
21. `modules/mission_planning/runtime/next_collab_line_runner.py`
22. `modules/mission_planning/runtime/aircraft_parallel_0303.py`
23. `modules/mission_planning/replanning/triggers/prior/pipeline.py`
24. `modules/mission_planning/replanning/triggers/imaging_schedule/pipeline.py`
25. `modules/mission_planning/replanning/triggers/path_deviation/pipeline.py`
26. `modules/mission_planning/replanning/triggers/attack/pipeline.py`
27. `modules/mission_planning/replanning/triggers/post_attack/pipeline.py`

## Reload Order Snapshot

현재 `PLANNER_RUNTIME_RELOAD_ORDER`:

1. `modules.mission_planning.MissionPlanner.AnS.coord_transform`
2. `modules.mission_planning.MissionPlanner.AnS.task_patterns_ver2`
3. `modules.mission_planning.MissionPlanner.AnS.mission_effectiveness_ver2`
4. `modules.mission_planning.MissionPlanner.AnS.env_patternselection`
5. `modules.mission_planning.MissionPlanner.AnS.mission_pipeline`
6. `modules.mission_planning.MissionPlanner.AnS`
7. `modules.mission_planning.pipelines.next_collab_path_builder`
8. `modules.mission_planning.runtime.next_collab_line_runner`
9. `modules.mission_planning.replanning.triggers.next_collab.pipeline`
10. `modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first`
11. `modules.mission_planning.replanning.triggers.recon_specialized.pipeline`
12. `modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan`
13. `modules.mission_planning.replanning.triggers.remaining_hybrid.current`
14. `modules.mission_planning.replanning.triggers.remaining_hybrid.general`
15. `modules.mission_planning.runtime.aircraft_parallel_0303`
16. `modules.mission_planning.replanning.triggers.prior.pipeline`
17. `modules.mission_planning.replanning.triggers.imaging_schedule.pipeline`
18. `modules.mission_planning.replanning.triggers.path_deviation.pipeline`
19. `modules.mission_planning.replanning.triggers.attack.pipeline`
20. `modules.mission_planning.replanning.triggers.post_attack.pipeline`

## Watched But Not in `PLANNER_RUNTIME_RELOAD_ORDER`

아래 파일은 source signature change를 유발하지만 `PLANNER_RUNTIME_RELOAD_ORDER`에서 직접 reload되는 module은 아니다.

- `modules/mission_planning/MissionPlanner/data_def/d0302.py`
- `modules/mission_planning/MissionPlanner/data_def/d0303.py`
- `modules/mission_planning/MissionPlanner/data_def/d0304.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py`
- `modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py`

`data_def.d0302/d0303/d0304` wrapper는 import 시 engine artifact builder module로 `sys.modules`를 교체한다. 그래서 GUI runtime build의 force-reload 경로에서 bare `data_def.d0302/d0303/d0304`를 reload하면 engine d0302/d0303/d0304 implementation이 간접 reload된다. 반면 engine d0301은 현재 GUI runtime build에서 직접 reload되는 bare module set에 포함되지 않고, source signature trigger와 artifact builder import identity 계약으로 감시된다.

## Binding Map Snapshot

`PIPELINE_RELOAD_BINDINGS`는 reload 후 GUI namespace에 재노출할 public helper 이름이다.

- `modules.mission_planning.MissionPlanner.AnS`
  - `run_divide_and_pattern`
  - `run_pulp_scheduling`
  - `build_mission_plan_0301`
  - `get_last_divide_and_pattern_metrics`
- `modules.mission_planning.replanning.triggers.prior.pipeline`
  - `run_prior_mission_pipeline`
  - `warm_prior_mission_pipeline`
  - `run_prior_post_rejoin_pipeline`
  - `warm_prior_post_rejoin_pipeline`
- `modules.mission_planning.replanning.triggers.imaging_schedule.pipeline`
  - `run_imaging_schedule_replan_pipeline`
  - `warm_imaging_schedule_replan_pipeline`
- `modules.mission_planning.replanning.triggers.path_deviation.pipeline`
  - `run_path_deviation_replan_pipeline`
  - `warm_path_deviation_replan_pipeline`
- `modules.mission_planning.replanning.triggers.next_collab.pipeline`
  - `run_next_collab_replan_pipeline`
  - `warm_next_collab_replan_pipeline`
- `modules.mission_planning.replanning.triggers.attack.pipeline`
  - `run_attack_exclusion_pipeline`
  - `run_attack_plan_pipeline`
  - `warm_attack_plan_pipeline`
- `modules.mission_planning.replanning.triggers.post_attack.pipeline`
  - `run_post_attack_rejoin_pipeline`
  - `warm_post_attack_rejoin_pipeline`

`remaining_hybrid`, `recon_specialized`, `aircraft_parallel_0303`는 `refresh_live_planning_helpers()`의 explicit rebinding block에서 별도 처리된다. 그 실제 rebinding fixture는 다음 TODO의 범위다.

## GUI Bridge Snapshot

`mission_planning_gui.py`는 planner runtime helper를 thin bridge로 사용한다.

- `_PLANNER_RUNTIME_WATCH_RELATIVE_PATHS = PLANNER_RUNTIME_WATCH_RELATIVE_PATHS`
- `_planner_runtime_source_signature()`는 `PROJECT_ROOT` 기준 source signature를 계산한다.
- `_refresh_live_planning_helpers()`는 `globals()`를 넘긴다.
- `_build_planner_runtime()`은 이전 signature가 있고 현재 signature와 다를 때만 `force_reload=True`로 간주한다.
- `_build_planner_runtime()`은 import path bootstrap 후 force reload 시 live helpers refresh를 호출한다.
- runtime build는 이후 bare `AnS`, `data_def.d0302/d0303/d0304`, `config`, `data_def.search_speed`를 import한다.
- force reload 시 GUI는 `AnS`, `data_def.d0302`, `data_def.d0303`, `data_def.d0304`, optional `config`, optional `data_def.search_speed`를 직접 reload한다.

## 검증 결과

명령:

```powershell
python "docs\mission planning refactoring\smoke_planner_hot_reload_snapshot.py"
```

결과:

```text
planner hot-reload watch/reload snapshot smoke ok
```

## 다음 지점

다음 미완료 TODO는 `planner hot-reload globals() rebinding fixture 작성`이다.
